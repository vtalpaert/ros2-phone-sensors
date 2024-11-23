import os
import threading

from flask import Flask, render_template
from flask_socketio import SocketIO, emit

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import TimeReference


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
        super().__init__("server_node")

        self.host_param = self.declare_parameter("host", "0.0.0.0")
        self.port_param = self.declare_parameter("port", 2000)
        self.debug_param = self.declare_parameter("debug", True)

        self.time_reference_interval_param = self.declare_parameter(
            "time_reference_interval", 20
        )  # [ms]

        self.time_reference_publisher = self.create_publisher(TimeReference, "time", 10)

    def log_message(self, message):
        self.get_logger().info(message)

    def handle_data(self, data):
        print(data)
        if "date_ms" in data:
            _floating_sec = float(data["date_ms"]) / 1000
            sec = int(_floating_sec)
            nanosec = int((_floating_sec - sec) * 1e9)
            msg = TimeReference()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.time_ref.sec = sec
            msg.time_ref.nanosec = nanosec
            msg.source = "phone"
            self.time_reference_publisher.publish(msg)


class ServerApp:
    def __init__(self, node):
        self.node = node

        # Get folders from install/ path
        template_folder = os.path.join(
            os.environ["COLCON_PREFIX_PATH"], "phone_as_a_robot", "lib", "templates"
        )
        static_folder = os.path.join(
            os.environ["COLCON_PREFIX_PATH"], "phone_as_a_robot", "lib", "static"
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
            emit("time_reference_set_interval", self.node.time_reference_interval_param.value)

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
        )


def main(args=None):
    rclpy.init(args=args)
    node = ServerNode()
    thread = StoppableThread(target=rclpy.spin, args=(node,), name="server_node")
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
