import os
import queue
import threading

import eventlet
from flask import Flask, render_template
from flask_socketio import SocketIO, emit

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import TimeReference, Imu, NavSatFix, Image, CameraInfo
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt8MultiArray
import cv_bridge

from .message_converters import (
    data_to_gnss_msgs,
    binary_to_image_msg,
    data_to_imu_msg,
    data_to_odometry_msg,
    data_to_time_reference_msg,
    yaml_to_camera_info,
)

package_name = "phone_sensors_bridge"


class StoppableThread(threading.Thread):
    """Thread class with a stop() method. The thread itself has to check
    regularly for the stopped() condition."""

    def __init__(self, *args, **kwargs):
        super(StoppableThread, self).__init__(*args, **kwargs)
        self._stop_event = threading.Event()

    def stop(self):
        self._stop_event.set()

    def stopped(self):
        return self._stop_event.is_set()


class ServerNode(Node):

    def __init__(self, usb_tx_sender=None):
        super().__init__(package_name + "_node")
        self.bridge = cv_bridge.CvBridge()
        self.name_param = self.declare_parameter("name", package_name)
        name = self.name_param.value

        self.host_param = self.declare_parameter("host", "0.0.0.0")
        self.port_param = self.declare_parameter("port", 2000)
        self.debug_param = self.declare_parameter("debug", False)
        self.secret_key_param = self.declare_parameter("secret_key", "secret!")
        self.ssl_certificate_param = self.declare_parameter(
            "ssl_certificate", "certs/certificate.crt"
        )
        self.ssl_private_key_param = self.declare_parameter(
            "ssl_private_key", "certs/private.key"
        )

        self.use_ros_time_param = self.declare_parameter("use_ros_time", False)

        self.time_reference_source_device_param = self.declare_parameter(
            "time_reference_source_device", "ros_to_device"
        )
        self.frame_id_imu_param = self.declare_parameter("frame_id_imu", name+ "_imu")
        self.frame_id_gnss_param = self.declare_parameter("frame_id_gnss", name+ "_gnss")
        self.frame_id_gnss_velocity_param = self.declare_parameter("frame_id_gnss_velocity", name + "_odom")
        self.frame_id_image_camera1_param = self.declare_parameter("frame_id_image_camera1", name + "_camera1")
        self.frame_id_image_camera2_param = self.declare_parameter("frame_id_image_camera2", name + "_camera2")
        self.time_reference_source_gnss_param = self.source_time_reference_param = (
            self.declare_parameter("time_reference_source_gnss", "device_to_gnss")
        )

        # IMU covariance parameters (from calibration script imu_gaussian_noise.py)
        self.imu_k_inflation_param = self.declare_parameter("imu_k_inflation", 10.0)                                  # safety factor
        self.imu_gyro_noise_density_param = self.declare_parameter("imu_gyro_noise_density", 7e-05)               # rad/s/sqrt(Hz)
        self.imu_gyro_quantization_variance_param = self.declare_parameter("imu_gyro_quantization_variance", 3e-07)  # rad²  (q²/12)
        self.imu_accel_noise_density_param = self.declare_parameter("imu_accel_noise_density", 2e-04)             # m/s²/sqrt(Hz)
        self.imu_accel_quantization_variance_param = self.declare_parameter("imu_accel_quantization_variance", 9e-04)  # (m/s²)²  (q²/12)
        self.imu_orient_variance_param = self.declare_parameter("imu_orient_variance", 2e-06)                     # rad²  (var_measured + var_quant)
        self.imu_orient_inflation_param = self.declare_parameter("imu_orient_inflation", 10.0)                    # safety factor on orientation variance
        self.gnss_position_inflation_param = self.declare_parameter("gnss_position_inflation", 10.0)              # safety factor on GPS position/altitude variance
        self.gnss_speed_variance_param = self.declare_parameter("gnss_speed_variance", 0.01)                      # (m/s)², GPS speed variance for case 1
        self.gnss_zero_velocity_variance_param = self.declare_parameter("gnss_zero_velocity_variance", 0.001)     # (m/s)², zero-velocity update variance for case 2

        # Declare all parameters to send to the client
        # Intervals are in [ms]
        self.client_params = self.declare_parameters(
            "",
            (
                ("time_reference_frequency", -1.0),
                ("imu_frequency", 50.0),  # 50 Hz for IMU
                ("gnss_watch_position", True),  # True enables GNSS via watchPosition, False disables GNSS
                (
                    "camera1_device_label",
                    "camera 1, facing front",
                ),  # In Firefox, use Facing front:1 and in Chrome use camera 1, facing front
                ("camera1_video_fps", 20.0),
                ("camera1_video_width", 720),  # Default to 720p resolution (horizontal)
                ("camera1_video_height", 720),  # Default to 720p resolution (vertical)
                ("camera1_video_compression", 0.3),
                (
                    "camera2_device_label",
                    "camera 0, facing back",
                ),  # Second camera device label
                ("camera2_video_fps", 20.0),
                ("camera2_video_width", 720),
                ("camera2_video_height", 720),
                ("camera2_video_compression", 0.3),
                ("usb_enabled", False),
                ("usb_device_type", "cdc"),  # "cdc" for native USB boards (Teensy, Leonardo), "cp2102" for USB-UART converter
                ("usb_baud", 115200),
            ),
        )

        fs = self.get_parameter("imu_frequency").value
        k = self.imu_k_inflation_param.value
        gyro_var = k * fs * self.imu_gyro_noise_density_param.value ** 2 + self.imu_gyro_quantization_variance_param.value
        accel_var = k * fs * self.imu_accel_noise_density_param.value ** 2 + self.imu_accel_quantization_variance_param.value
        orient_var = self.imu_orient_inflation_param.value * self.imu_orient_variance_param.value
        self._angular_velocity_covariance = [gyro_var, 0.0, 0.0, 0.0, gyro_var, 0.0, 0.0, 0.0, gyro_var]
        self._linear_acceleration_covariance = [accel_var, 0.0, 0.0, 0.0, accel_var, 0.0, 0.0, 0.0, accel_var]
        self._orientation_covariance = [orient_var, 0.0, 0.0, 0.0, orient_var, 0.0, 0.0, 0.0, orient_var]

        self.time_reference_device_publisher = self.create_publisher(
            TimeReference, "time/device", 10
        )
        self.imu_publisher = self.create_publisher(Imu, "imu", 10)
        self.gnss_publisher = self.create_publisher(NavSatFix, "gnss", 10)
        self.time_reference_gnss_publisher = self.create_publisher(
            TimeReference, "time/gnss", 10
        )
        self.odometry_publisher = self.create_publisher(Odometry, "gnss/odometry", 10)
        self.video_camera1_publisher = self.create_publisher(Image, "camera1/image_raw", 10)
        self.video_camera2_publisher = self.create_publisher(Image, "camera2/image_raw", 10)
        self.usb_rx_publisher = self.create_publisher(UInt8MultiArray, "usb/rx", 10)
        if usb_tx_sender is not None:
            self.usb_tx_subscriber = self.create_subscription(
                UInt8MultiArray, "usb/tx",
                lambda msg: usb_tx_sender(bytes(msg.data)),
                10,
            )

        # Load camera calibration if file exists for camera1
        self.camera1_calibration_file = self.declare_parameter(
            "camera1_calibration_file", ""
        ).value
        self.camera1_info_msg = None
        if self.camera1_calibration_file and os.path.exists(
            self.camera1_calibration_file
        ):
            try:
                self.camera1_info_msg = yaml_to_camera_info(self.camera1_calibration_file)
                self.get_logger().info(
                    f"Loaded camera1 calibration from {self.camera1_calibration_file}"
                )
                self.camera1_info_publisher = self.create_publisher(
                    CameraInfo, "camera1/camera_info", 10
                )
            except Exception as e:
                self.get_logger().error(f"Failed to load camera1 calibration: {str(e)}")

        # Load camera calibration if file exists for camera2
        self.camera2_calibration_file = self.declare_parameter(
            "camera2_calibration_file", ""
        ).value
        self.camera2_info_msg = None
        if self.camera2_calibration_file and os.path.exists(
            self.camera2_calibration_file
        ):
            try:
                self.camera2_info_msg = yaml_to_camera_info(self.camera2_calibration_file)
                self.get_logger().info(
                    f"Loaded camera2 calibration from {self.camera2_calibration_file}"
                )
                self.camera2_info_publisher = self.create_publisher(
                    CameraInfo, "camera2/camera_info", 10
                )
            except Exception as e:
                self.get_logger().error(f"Failed to load camera2 calibration: {str(e)}")

    def log_debug(self, message):
        self.get_logger().debug(message)

    def log_info(self, message):
        self.get_logger().info(message)

    def log_warning(self, message):
        self.get_logger().warning(message)

    def log_error(self, message):
        self.get_logger().error(message)

    def handle_binary_frame(self, data, camera_id):
        if camera_id == "camera1":
            frame_id = self.frame_id_image_camera1_param.value
            publisher = self.video_camera1_publisher
            camera_info_msg = self.camera1_info_msg
            camera_info_publisher = getattr(self, 'camera1_info_publisher', None)
        elif camera_id == "camera2":
            frame_id = self.frame_id_image_camera2_param.value
            publisher = self.video_camera2_publisher
            camera_info_msg = self.camera2_info_msg
            camera_info_publisher = getattr(self, 'camera2_info_publisher', None)
        else:
            self.get_logger().warning(f"Unknown camera_id: {camera_id}")
            return
        try:
            ros_time = (
                self.get_clock().now().to_msg()
                if self.use_ros_time_param.value
                else False
            )
            img_msg = binary_to_image_msg(data, ros_time, frame_id, self.bridge)
            publisher.publish(img_msg)
            if camera_info_msg and camera_info_publisher:
                camera_info_msg.header.stamp = img_msg.header.stamp
                camera_info_msg.header.frame_id = img_msg.header.frame_id
                camera_info_publisher.publish(camera_info_msg)
        except ValueError as e:
            self.get_logger().warning(f"Failed to convert binary frame from {camera_id}: {str(e)}")

    def handle_data(self, data):
        if "video_frame" not in data and "motion" not in data:
            self.log_debug(str(data))
        # currently there is no message that includes several sensors at the same time,
        # so an if-else logic is sufficient
        if "loc" in data:
            ros_time = (
                self.get_clock().now().to_msg()
                if self.use_ros_time_param.value
                else False
            )
            try:
                fix, time = data_to_gnss_msgs(
                    data,
                    ros_time,
                    self.frame_id_gnss_param.value,
                    self.time_reference_source_gnss_param.value,
                    self.gnss_position_inflation_param.value,
                )
                self.gnss_publisher.publish(fix)
                self.time_reference_gnss_publisher.publish(time)
                odom = data_to_odometry_msg(
                    data, ros_time, self.frame_id_gnss_velocity_param.value,
                    self.gnss_speed_variance_param.value,
                    self.gnss_zero_velocity_variance_param.value,
                )
                if odom is not None:
                    self.odometry_publisher.publish(odom)
            except (TypeError, ValueError) as e:
                self.get_logger().warning(
                    f"Failed to convert GNSS data: {str(e)} in {data}"
                )
        elif "motion" in data:
            ros_time = (
                self.get_clock().now().to_msg()
                if self.use_ros_time_param.value
                else False
            )
            try:
                msg = data_to_imu_msg(
                    data, ros_time, self.frame_id_imu_param.value,
                    self._orientation_covariance,
                    self._angular_velocity_covariance,
                    self._linear_acceleration_covariance,
                )
                self.imu_publisher.publish(msg)
            except (TypeError, ValueError) as e:
                self.get_logger().warning(
                    f"Failed to convert IMU data: {str(e)} in {data}"
                )
        elif "device_info" in data:
            camera_id = data.get("camera_id", "camera1")
            self.get_logger().info(
                "Detected %s device with label %s" % (camera_id, data["device_info"]["label"])
            )
        else:
            msg = data_to_time_reference_msg(
                data,
                self.get_clock().now(),
                self.time_reference_source_device_param.value,
            )
            self.time_reference_device_publisher.publish(msg)


class ServerApp:
    def __init__(self):
        self._usb_tx_queue = queue.Queue()
        self.node = ServerNode(usb_tx_sender=self._usb_tx_queue.put)

        # Get folders from install/ path
        template_folder = os.path.join(
            os.environ["COLCON_PREFIX_PATH"], package_name, "lib", "templates"
        )
        static_folder = os.path.join(
            os.environ["COLCON_PREFIX_PATH"], package_name, "lib", "static"
        )

        self.app = Flask(
            __name__, template_folder=template_folder, static_folder=static_folder
        )
        self.app.config["SECRET_KEY"] = self.node.secret_key_param.value
        self.socketio = SocketIO(self.app)

        print(
            f"Running with template folder {template_folder} and static folder {static_folder}. Async mode is {self.socketio.async_mode}"
        )

        @self.app.route("/")
        def index():
            return render_template(
                "index.html",
                async_mode=self.socketio.async_mode,
            )

        @self.app.route("/test-video-permissions")
        def test_permissions():
            return render_template("test_video_permissions.html")

        @self.app.route("/test-web-usb")
        def test_web_usb():
            return render_template("test_web_usb.html")

        @self.app.route("/test-serial")
        def test_serial():
            return render_template("test_serial.html")

        @self.socketio.on("connect")
        def handle_connect_event():
            sleep_time = 0.1  # Short delay to ensure parameters are processed before starting stream
            # Stop any ongoing streams before reconfiguring
            emit("stream_stop")
            eventlet.sleep(sleep_time)
            # Configure client based on ROS parameters
            for param in self.node.client_params:
                emit(
                    param.name,
                    param.value,
                )
            eventlet.sleep(sleep_time)
            # Signal client that all parameters have been delivered
            emit("stream_start")

        @self.socketio.on("debug")
        def handle_debug_event(message):
            self.node.log_debug(message)

        @self.socketio.on("info")
        def handle_info_event(message):
            self.node.log_info(message)

        @self.socketio.on("warn")
        def handle_warn_event(message):
            self.node.log_warning(message)

        @self.socketio.on("error")
        def handle_error_event(message):
            self.node.log_error(message)

        @self.socketio.on("data")
        def handle_data_event(data):
            self.node.handle_data(data)

        @self.socketio.on("camera1_frame")
        def handle_camera1_frame(data):
            self.node.handle_binary_frame(data, "camera1")

        @self.socketio.on("camera2_frame")
        def handle_camera2_frame(data):
            self.node.handle_binary_frame(data, "camera2")

        @self.socketio.on("latency_ping")
        def handle_latency_ping_event(data):
            t_server_ms = self.node.get_clock().now().nanoseconds // 1_000_000
            emit("latency_pong", {
                "t1": data["t1"],
                "t_server_ms": t_server_ms,
            })

        @self.socketio.on("usb_rx_data")
        def handle_usb_rx_data(data):
            msg = UInt8MultiArray()
            msg.data = list(data)
            self.node.usb_rx_publisher.publish(msg)

        def usb_tx_background_worker():
            while True:
                self.socketio.sleep(0.005)
                while not self._usb_tx_queue.empty():
                    try:
                        data = self._usb_tx_queue.get_nowait()
                        self.socketio.emit("usb_tx_data", data, namespace="/")
                    except queue.Empty:
                        break

        self.socketio.start_background_task(usb_tx_background_worker)

    def run(self):
        if not os.path.exists(
            self.node.ssl_certificate_param.value
        ) or not os.path.exists(self.node.ssl_private_key_param.value):
            print("Missing SSL certificates")
        else:
            print("Found SSL certificates. Starting server")
            self.socketio.run(
                self.app,
                self.node.host_param.value,
                self.node.port_param.value,
                debug=self.node.debug_param.value,
                use_reloader=False,
                certfile=self.node.ssl_certificate_param.value,
                keyfile=self.node.ssl_private_key_param.value,
            )


def main(args=None):
    rclpy.init(args=args)
    app = ServerApp()
    thread = StoppableThread(
        target=rclpy.spin, args=(app.node,), name=package_name + "_node_thread"
    )
    try:
        thread.start()
        app.run()
    finally:
        thread.stop()
        app.node.destroy_node()
        # rclpy.shutdown()
        thread.join()


if __name__ == "__main__":
    main()
