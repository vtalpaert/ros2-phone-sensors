import os
import threading

from flask import Flask, render_template
from flask_socketio import SocketIO, emit

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import TimeReference
from sensor_msgs.msg import Imu

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
        self.name_param = self.declare_parameter("name", package_name)
        name = self.name_param.value

        self.host_param = self.declare_parameter("host", "0.0.0.0")
        self.port_param = self.declare_parameter("port", 2000)
        self.debug_param = self.declare_parameter("debug", True)

        # TODO add a parameter to use ROS time instead of device time

        self.time_reference_source_device_param = self.declare_parameter(
            "time_reference_source_device", "ros_to_device"
        )
        self.frame_id_imu_param = self.declare_parameter("frame_id_imu", name)
        self.frame_id_gnss_param = self.declare_parameter("frame_id_gnss", name)
        self.time_reference_source_gnss_param = self.source_time_reference_param = (
            self.declare_parameter("time_reference_source_gnss", "device_to_gnss")
        )

        # Declare all parameters to send to the client
        # Intervals are in [ms]
        self.client_params = self.declare_parameters(
            "",
            (
                ("time_reference_set_interval", -1),
                ("imu_set_interval", -1),
                ("gnss_set_interval", 1000),
            ),
        )

        self.time_reference_publisher = self.create_publisher(TimeReference, "time", 10)
        self.imu_publisher = self.create_publisher(Imu, "imu", 10)

    def log_message(self, message):
        self.get_logger().info(message)

    def handle_data(self, data):
        if "gnss" in data:
            data_to_gnss_msgs(
                data,
                self.frame_id_gnss_param.value,
                self.time_reference_source_gnss_param.value,
            )
        elif "imu" in data:
            msg = data_to_imu_msg(data, self.frame_id_imu_param.value)
            self.imu_publisher.publish(msg)
        else:
            msg = data_to_time_reference_msg(
                data,
                self.get_clock().now(),
                self.time_reference_source_device_param.value,
            )
            self.time_reference_publisher.publish(msg)


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

        @self.socketio.on("connect")
        def handle_connect_event():
            # Configure client based on ROS parameters
            for param in self.node.client_params:
                emit(
                    param.name,
                    param.value,
                )

        @self.socketio.on("log")
        def handle_log_event(message):
            self.node.log_message(message)

        @self.socketio.on("data")
        def handle_data_event(data):
            self.node.handle_data(data)

    def run(self):
        self.socketio.run(
            self.app,
            self.node.host_param.value,
            self.node.port_param.value,
            debug=self.node.debug_param.value,
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
        thread.join()
        # node.destroy_node()
        # rclpy.shutdown()


if __name__ == "__main__":
    main()
