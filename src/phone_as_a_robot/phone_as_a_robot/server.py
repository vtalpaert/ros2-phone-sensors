import os
import threading

from flask import Flask, render_template
from flask_socketio import SocketIO

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


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


class Server(Node):

    def __init__(self):
        super().__init__("server")

        self.host_param = self.declare_parameter("host", "0.0.0.0")
        self.port_param = self.declare_parameter("port", 2000)
        self.debug_param = self.declare_parameter("debug", True)

        self.publisher_ = self.create_publisher(String, "topic", 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = "Hello World: %d" % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    node = Server()
    rclpy.spin_once(node)
    thread = StoppableThread(target=rclpy.spin, args=(node,), name="server_node")

    templates_directory = os.path.join(
        os.environ["COLCON_PREFIX_PATH"], "phone_as_a_robot", "lib", "templates"
    )
    app = Flask(__name__, template_folder=templates_directory)
    app.config["SECRET_KEY"] = "secret!"
    socketio = SocketIO(app)

    @app.route("/")
    def index():
        return render_template(
            "index.html",
            async_mode=socketio.async_mode,
        )

    @socketio.on("my event", namespace="/test")
    def handle_my_custom_namespace_event(json):
        print("received json: " + str(json))

    try:
        thread.start()
        socketio.run(
            app,
            node.host_param.value,
            node.port_param.value,
            debug=node.debug_param.value,
        )
    finally:
        thread.stop()
        thread.join()
        # node.destroy_node()
        # rclpy.shutdown()


if __name__ == "__main__":
    main()
