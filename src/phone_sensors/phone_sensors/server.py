import os
import threading

from flask import Flask, render_template
from flask_socketio import SocketIO, emit

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import TimeReference, Imu, NavSatFix, Image
import cv_bridge

from .message_converters import *

package_name = "phone_sensors"


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

    def __init__(self):
        super().__init__(package_name + "_node")
        self.bridge = cv_bridge.CvBridge()
        self.name_param = self.declare_parameter("name", package_name)
        name = self.name_param.value

        self.host_param = self.declare_parameter("host", "0.0.0.0")
        self.port_param = self.declare_parameter("port", 2000)
        self.debug_param = self.declare_parameter("debug", True)

        self.use_ros_time_param = self.declare_parameter("use_ros_time", False)

        self.time_reference_source_device_param = self.declare_parameter(
            "time_reference_source_device", "ros_to_device"
        )
        self.frame_id_imu_param = self.declare_parameter("frame_id_imu", name)
        self.frame_id_gnss_param = self.declare_parameter("frame_id_gnss", name)
        self.frame_id_image_param = self.declare_parameter("frame_id_image", name)
        self.time_reference_source_gnss_param = self.source_time_reference_param = (
            self.declare_parameter("time_reference_source_gnss", "device_to_gnss")
        )

        # Declare all parameters to send to the client
        # Intervals are in [ms]
        self.client_params = self.declare_parameters(
            "",
            (
                ("time_reference_frequency", -1.0),
                ("imu_frequency", 100.0),  # 100 Hz for IMU
                ("gnss_frequency", 10.0),  # 10 Hz for GNSS
                ("camera_device_label", "Facing front:1"),
                ("show_video_preview", True),
                ("video_fps", 30.0),
                ("video_width", 1280),  # Default to 720p resolution (horizontal)
                ("video_height", 720),   # Default to 720p resolution (vertical)
                ("video_compression", 0.3),
            ),
        )

        self.time_reference_device_publisher = self.create_publisher(
            TimeReference, "time/device", 10
        )
        self.imu_publisher = self.create_publisher(Imu, "imu", 10)
        self.gnss_publisher = self.create_publisher(NavSatFix, "gnss", 10)
        self.time_reference_gnss_publisher = self.create_publisher(
            TimeReference, "time/gnss", 10
        )
        self.video_publisher = self.create_publisher(Image, "camera/image_raw", 10)

    def log_debug(self, message):
        self.get_logger().debug(message)

    def log_info(self, message):
        self.get_logger().info(message)

    def log_error(self, message):
        self.get_logger().error(message)

    def handle_data(self, data):
        if "gnss" in data:
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
                )
                self.gnss_publisher.publish(fix)
                self.time_reference_gnss_publisher.publish(time)
            except (TypeError, ValueError) as e:
                self.get_logger().warning(f"Failed to convert GNSS data: {str(e)}")
        elif "imu" in data:
            ros_time = (
                self.get_clock().now().to_msg()
                if self.use_ros_time_param.value
                else False
            )
            try:
                msg = data_to_imu_msg(data, ros_time, self.frame_id_imu_param.value)
                self.imu_publisher.publish(msg)
            except (TypeError, ValueError) as e:
                self.get_logger().warning(f"Failed to convert IMU data: {str(e)}")
        elif "device_info" in data:
            self.get_logger().info(
                "Detected camera device with label %s" % data["device_info"]["label"]
            )
        elif "video_frame" in data:
            try:
                img_msg = data_to_image_msg(
                    data,
                    self.bridge,
                    self.frame_id_image_param.value,
                    self.get_clock().now().to_msg(),
                )
                self.video_publisher.publish(img_msg)
            except ValueError as e:
                self.get_logger().warning(f"Failed to convert video frame: {str(e)}")
        else:
            msg = data_to_time_reference_msg(
                data,
                self.get_clock().now(),
                self.time_reference_source_device_param.value,
            )
            self.time_reference_device_publisher.publish(msg)


class ServerApp:
    def __init__(self, node):
        self.node = node

        # Get folders from install/ path
        template_folder = os.path.join(
            os.environ["COLCON_PREFIX_PATH"], package_name, "lib", "templates"
        )
        static_folder = os.path.join(
            os.environ["COLCON_PREFIX_PATH"], package_name, "lib", "static"
        )
        print(
            "Running with template folder %s and static folder %s"
            % (template_folder, static_folder)
        )
        self.app = Flask(
            __name__, template_folder=template_folder, static_folder=static_folder
        )
        self.app.config["SECRET_KEY"] = "secret!"
        self.socketio = SocketIO(self.app)

        @self.app.route("/")
        def index():
            return render_template(
                "index.html",
                async_mode=self.socketio.async_mode,
            )

        @self.app.route("/test-video-permissions")
        def test_permissions():
            return render_template("test_video_permissions.html")

        @self.socketio.on("connect")
        def handle_connect_event():
            # Configure client based on ROS parameters
            for param in self.node.client_params:
                emit(
                    param.name,
                    param.value,
                )

        @self.socketio.on("debug")
        def handle_debug_event(message):
            self.node.log_debug(message)

        @self.socketio.on("info")
        def handle_info_event(message):
            self.node.log_info(message)

        @self.socketio.on("error")
        def handle_error_event(message):
            self.node.log_error(message)

        @self.socketio.on("data")
        def handle_data_event(data):
            self.node.handle_data(data)

    def run(self):
        self.socketio.run(
            self.app,
            self.node.host_param.value,
            self.node.port_param.value,
            debug=self.node.debug_param.value,
            use_reloader=False,
            ssl_context="adhoc",
        )


def main(args=None):
    rclpy.init(args=args)
    node = ServerNode()
    thread = StoppableThread(
        target=rclpy.spin, args=(node,), name=package_name + "_node_thread"
    )
    app = ServerApp(node)
    try:
        thread.start()
        app.run()
    finally:
        thread.stop()
        node.destroy_node()
        # rclpy.shutdown()
        thread.join()


if __name__ == "__main__":
    main()
