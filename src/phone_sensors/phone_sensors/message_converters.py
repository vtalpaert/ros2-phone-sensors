import math

from sensor_msgs.msg import TimeReference
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix, NavSatStatus


def millisec_to_sec_nanosec(ms):
    # Typical input {'date_ms': 1732380161698}
    _floating_sec = float(ms) / 1000
    sec = int(_floating_sec)
    nanosec = int((_floating_sec - sec) * 1e9)
    return sec, nanosec


def euler_to_quaternion(roll, pitch, yaw):
    # Don't import numpy just for this
    roll = math.radians(roll)
    pitch = math.radians(pitch)
    yaw = math.radians(yaw)
    qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(
        roll / 2
    ) * math.sin(pitch / 2) * math.sin(yaw / 2)
    qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(
        roll / 2
    ) * math.cos(pitch / 2) * math.sin(yaw / 2)
    qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(
        roll / 2
    ) * math.sin(pitch / 2) * math.cos(yaw / 2)
    qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(
        roll / 2
    ) * math.sin(pitch / 2) * math.sin(yaw / 2)
    return qx, qy, qz, qw


def data_to_time_reference_msg(data, ros_time, source):
    # Typical input {'date_ms': 1732380161698}
    sec, nanosec = millisec_to_sec_nanosec(data["date_ms"])
    msg = TimeReference()
    msg.header.stamp = ros_time.to_msg()
    msg.time_ref.sec = sec
    msg.time_ref.nanosec = nanosec
    msg.source = source
    return msg


def data_to_imu_msg(data, frame_id):
    # Typical input {'date_ms': 1732380161741,
    # 'imu': {'ax': 0.5315127968788147, 'ay': 4.6950297355651855, 'az': 8.413225173950195,
    # 'gx': 5.914999961853027, 'gy': 3.325000047683716, 'gz': 2.0300002098083496,
    # 'ox': 28.488818648731325, 'oy': -3.708893076611099, 'oz': 108.19609673425123,
    # 'motion_interval_ms': 100, 'absolute': True}}
    msg = Imu()
    # Here we will assume the motion delay applies to orientation delay
    # Note that on the client side, motion and orientation are handled separately
    delay_ms = data["imu"]["motion_interval_ms"]
    sec, nanosec = millisec_to_sec_nanosec(data["date_ms"] - delay_ms)
    msg.header.stamp.sec = sec
    msg.header.stamp.nanosec = nanosec
    msg.header.frame_id = frame_id
    _is_absolute = data["imu"]["absolute"]
    # Here the interpretation of axis is dependant on both the device
    # but also the use case. Re-orientation should be handled in your tf tree
    qx, qy, qz, qw = euler_to_quaternion(
        data["imu"]["ox"], data["imu"]["oy"], data["imu"]["oz"]
    )
    msg.orientation.x = float(qx)
    msg.orientation.y = float(qy)
    msg.orientation.z = float(qz)
    msg.orientation.w = float(qw)
    msg.orientation_covariance = 9 * [0.0]
    msg.angular_velocity.x = float(data["imu"]["gx"])
    msg.angular_velocity.y = float(data["imu"]["gy"])
    msg.angular_velocity.z = float(data["imu"]["gz"])
    msg.angular_velocity_covariance = 9 * [0.0]
    msg.linear_acceleration.x = float(data["imu"]["ax"])
    msg.linear_acceleration.y = float(data["imu"]["ay"])
    msg.linear_acceleration.z = float(data["imu"]["az"])
    msg.linear_acceleration_covariance = 9 * [0.0]
    return msg


def data_to_gnss_msgs(data, frame_id, source):
    # Typical input with low accuracy {'date_ms': 1732386620779,
    # 'gnss': {'coords': {'latitude': 48.8868302, 'longitude': 2.3602171, 'altitude': 95.30000305175781,
    # 'accuracy': 100, 'altitudeAccuracy': 100,
    # 'heading': None, 'speed': None},
    # 'timestamp': 1732386620774}}

    fix = NavSatFix()
    # header.stamp specifies the "ROS" time for this measurement (the
    # corresponding satellite time may be reported using the
    # sensor_msgs/TimeReference message).
    sec, nanosec = millisec_to_sec_nanosec(data["date_ms"])
    fix.header.stamp.sec = sec
    fix.header.stamp.nanosec = nanosec
    fix.header.frame_id = frame_id
    fix.status.status = NavSatStatus.STATUS_FIX  # unaugmented fix
    fix.status.service = 0  # unknown
    fix.latitude = float(data["gnss"]["coords"]["latitude"])
    fix.longitude = float(data["gnss"]["coords"]["longitude"])
    fix.altitude = float(data["gnss"]["coords"]["altitude"])
    fix.position_covariance = (
        float(data["gnss"]["coords"]["accuracy"]),
        0.0,
        0.0,
        0.0,
        float(data["gnss"]["coords"]["accuracy"]),
        0.0,
        0.0,
        0.0,
        float(data["gnss"]["coords"]["altitudeAccuracy"]),
    )
    fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

    # Do something with heading and speed
    # Beware that heading is North-West instead of Est-North-Up
    # (ENU is the normal use in robotics)
    # https://developer.mozilla.org/en-US/docs/Web/API/GeolocationCoordinates

    time = TimeReference()
    time.header.stamp = fix.header.stamp
    ts_sec, ts_nanosec = millisec_to_sec_nanosec(data["gnss"]["timestamp"])
    time.time_ref.sec = ts_sec
    time.time_ref.nanosec = ts_nanosec
    time.source = source

    print(fix, time)
