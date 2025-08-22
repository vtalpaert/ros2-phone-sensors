import math
import yaml
import base64
import numpy as np
import cv2

from sensor_msgs.msg import TimeReference
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix, NavSatStatus
from sensor_msgs.msg import CameraInfo


def millisec_to_sec_nanosec(ms):
    # Typical input {'date_ms': 1732380161698}
    _floating_sec = float(ms) / 1000
    sec = int(_floating_sec)
    nanosec = int((_floating_sec - sec) * 1e9)
    return sec, nanosec


def get_quaternion_from_euler(roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.
    Author: AutomaticAddison.com

    Input
        :param roll: The roll (rotation around x-axis) angle in radians.
        :param pitch: The pitch (rotation around y-axis) angle in radians.
        :param yaw: The yaw (rotation around z-axis) angle in radians.

    Output
        :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(
        roll / 2
    ) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(
        roll / 2
    ) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(
        roll / 2
    ) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(
        roll / 2
    ) * np.sin(pitch / 2) * np.sin(yaw / 2)
    return [qx, qy, qz, qw]


def data_to_time_reference_msg(data, ros_time, source):
    # Typical input {'date_ms': 1732380161698}
    sec, nanosec = millisec_to_sec_nanosec(data["date_ms"])
    msg = TimeReference()
    msg.header.stamp = ros_time.to_msg()
    msg.time_ref.sec = sec
    msg.time_ref.nanosec = nanosec
    msg.source = source
    return msg


def data_to_imu_msg(data, ros_time, frame_id):
    # Typical input {'date_ms': 1746375486895,
    # 'motion': {'ax': 0.44434577226638794, 'ay': -0.5894360542297363, 'az': 2.010622024536133,
    # 'rb': 13.755000114440918, 'rg': 20.96500015258789, 'ra': -55.05500030517578,
    # 'ob': 33.10600160962329, 'og': -5.509307511590301, 'oa': 101.09377997193032,
    # 'im': 100, 'abs': True}}
    msg = Imu()
    # Here we have access to the refresh interval for device motion,
    # it may be used to estimate the delay, or "how old" is the motion data
    # Note that on the client side, motion and orientation are handled separately,
    # so the motion delay is not equal to the orientation delay
    # _motion_interval_ms = data["motion"]["im"]
    if not ros_time:
        # Using data["date_ms"] - _motion_interval_ms is possible for precision
        # but it needs to be tested. Indeed there is no guarantee that the refresh
        # rate is equal to the delay
        sec, nanosec = millisec_to_sec_nanosec(data["date_ms"])
        msg.header.stamp.sec = sec
        msg.header.stamp.nanosec = nanosec
    else:
        msg.header.stamp = ros_time
    msg.header.frame_id = frame_id
    # _is_absolute = data["motion"]["abs"]
    # Re-orientation should be handled in your tf tree
    # Orientation is in degree, in ENU
    # alpha around z, beta around x, gamma around y
    # Orientation angle rotations should always be done in a Z - X' - Y'' order
    orientation_alpha = data["motion"]["oa"]
    orientation_beta = data["motion"]["ob"]
    orientation_gamma = data["motion"]["og"]
    qx, qy, qz, qw = get_quaternion_from_euler(
        math.radians(orientation_beta),
        math.radians(orientation_gamma),
        math.radians(orientation_alpha),
    )
    msg.orientation.x = float(qx)
    msg.orientation.y = float(qy)
    msg.orientation.z = float(qz)
    msg.orientation.w = float(qw)
    msg.orientation_covariance = 9 * [0.0]

    # Rotation rate is in deg/sec, in the device coordinates frame
    # alpha around z, beta around x, gamma around y
    rotation_alpha = data["motion"]["ra"]
    rotation_beta = data["motion"]["rb"]
    rotation_gamma = data["motion"]["rg"]
    msg.angular_velocity.x = float(math.radians(rotation_beta))
    msg.angular_velocity.y = float(math.radians(rotation_gamma))
    msg.angular_velocity.z = float(math.radians(rotation_alpha))
    msg.angular_velocity_covariance = 9 * [0.0]

    # Acceleration is in m/s2, in the device coordinates frame
    msg.linear_acceleration.x = float(data["motion"]["ax"])
    msg.linear_acceleration.y = float(data["motion"]["ay"])
    msg.linear_acceleration.z = float(data["motion"]["az"])
    msg.linear_acceleration_covariance = 9 * [0.0]
    return msg


def data_to_gnss_msgs(data, ros_time, frame_id, source):
    # Typical input with low accuracy {'date_ms': 1732386620779,
    # 'loc': {'coords': {'latitude': 48.88xxxxx, 'longitude': 2.36xxxxx,
    # 'altitude': 95.30000305175781,
    # 'accuracy': 100, 'altitudeAccuracy': 100,
    # 'heading': None, 'speed': None},
    # 'timestamp': 1732386620774}}

    fix = NavSatFix()
    # From the documentation, header.stamp specifies the "ROS" time for this measurement
    # (the corresponding satellite time may be reported using the
    # sensor_msgs/TimeReference message)
    # In our case, mobile phones are synchronised to the millisecond. So we want to
    # report on the precise timestamp at which the geolocation is measured
    # This behaviour is overwritten with use_ros_time parameter
    if not ros_time:
        sec, nanosec = millisec_to_sec_nanosec(data["loc"]["timestamp"])
        fix.header.stamp.sec = sec
        fix.header.stamp.nanosec = nanosec
    else:
        fix.header.stamp = ros_time
    fix.header.frame_id = frame_id
    fix.status.status = NavSatStatus.STATUS_FIX  # unaugmented fix
    fix.status.service = 0  # unknown
    fix.latitude = float(data["loc"]["coords"]["latitude"])
    fix.longitude = float(data["loc"]["coords"]["longitude"])
    # Handle None altitude by defaulting to 0
    altitude = data["loc"]["coords"]["altitude"]
    fix.altitude = float(altitude) if altitude is not None else 0.0
    altitude_accuracy = data["loc"]["coords"]["altitudeAccuracy"]
    altitude_accuracy = (
        float(altitude_accuracy) if altitude_accuracy is not None else 0.0
    )
    fix.position_covariance = (
        float(data["loc"]["coords"]["accuracy"]),
        0.0,
        0.0,
        0.0,
        float(data["loc"]["coords"]["accuracy"]),
        0.0,
        0.0,
        0.0,
        altitude_accuracy,
    )
    fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

    # Do something with heading and speed
    # Beware that heading is North-West instead of Est-North-Up
    # (ENU is the normal use in robotics)
    # https://developer.mozilla.org/en-US/docs/Web/API/GeolocationCoordinates

    # Report the time difference between the device time and device GNSS timestamp
    # or the device time to ROS time
    time = TimeReference()
    sec, nanosec = millisec_to_sec_nanosec(data["date_ms"])
    time.header.stamp.sec = sec
    time.header.stamp.nanosec = nanosec
    time.time_ref = fix.header.stamp
    time.source = source

    return fix, time


def yaml_to_camera_info(yaml_fname):
    """Parse camera calibration data from a YAML file into a CameraInfo message.

    Parameters
    ----------
    yaml_fname : str
        Path to YAML file containing camera calibration data

    Returns
    -------
    camera_info_msg : sensor_msgs.msg.CameraInfo
        CameraInfo message containing the calibration data
    """
    with open(yaml_fname, "r") as f:
        calib_data = yaml.safe_load(f)

    msg = CameraInfo()
    msg.width = calib_data["image_width"]
    msg.height = calib_data["image_height"]
    msg.k = calib_data["camera_matrix"]["data"]
    msg.d = calib_data["distortion_coefficients"]["data"]
    msg.r = calib_data["rectification_matrix"]["data"]
    msg.p = calib_data["projection_matrix"]["data"]
    msg.distortion_model = calib_data["distortion_model"]
    return msg


def data_to_image_msg(data, bridge, frame_id, ros_time):
    """Convert base64 image data to ROS Image message"""
    # Validate input data
    if not data.get("video_frame"):
        raise ValueError("Empty video frame data")

    # Split and validate base64 data
    parts = data["video_frame"].split(",")
    if len(parts) != 2:
        raise ValueError("Invalid base64 image format")

    # Decode base64 image
    img_data = base64.b64decode(parts[1])
    if not img_data:
        raise ValueError("Failed to decode base64 data")

    # Convert to numpy array
    nparr = np.frombuffer(img_data, np.uint8)
    if nparr.size == 0:
        raise ValueError("Empty image buffer")

    # Decode image
    frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
    if frame is None:
        raise ValueError("Failed to decode image")

    # Convert to ROS Image message
    img_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
    if ros_time is None:
        # Use the timestamp from the data
        sec, nanosec = millisec_to_sec_nanosec(data["date_ms"])
        img_msg.header.stamp.sec = sec
        img_msg.header.stamp.nanosec = nanosec
    else:
        img_msg.header.stamp = ros_time
    img_msg.header.frame_id = frame_id
    return img_msg
