import collections
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8MultiArray
from pySerialTransfer import pySerialTransfer as txfer


class ROSStream:
    """Duck-type replacement for serial.Serial, backed by ROS usb/tx and usb/rx topics.

    SerialTransfer accesses: is_open, open(), close(), write(data), read(size), in_waiting.
    """

    def __init__(self, node: Node):
        self._pub = node.create_publisher(UInt8MultiArray, 'usb/tx', 10)
        node.create_subscription(UInt8MultiArray, 'usb/rx', self._rx_callback, 10)
        self._rx_buffer = collections.deque()
        self._lock = threading.Lock()
        self.is_open = False

    def _rx_callback(self, msg: UInt8MultiArray):
        with self._lock:
            self._rx_buffer.extend(msg.data)

    def open(self):
        self.is_open = True

    def close(self):
        self.is_open = False

    def write(self, data: bytes) -> int:
        self._pub.publish(UInt8MultiArray(data=list(data)))
        return len(data)

    def read(self, size: int) -> bytes:
        with self._lock:
            n = min(size, len(self._rx_buffer))
            return bytes(self._rx_buffer.popleft() for _ in range(n))

    @property
    def in_waiting(self) -> int:
        with self._lock:
            return len(self._rx_buffer)


class CmdVelToArduino(Node):

    def __init__(self):
        super().__init__('cmd_vel_to_arduino')
        self._stream = ROSStream(self)

        # Bypass SerialTransfer's serial.Serial construction:
        # restrict_ports=False skips port validation, then we replace
        # link.connection with our ROS-backed stream before open() is called.
        self._link = txfer.SerialTransfer('', restrict_ports=False)
        self._link.connection = self._stream
        self._link.open()

        self.create_subscription(Twist, 'cmd_vel', self._cmd_vel_callback, 10)
        # Poll for incoming packets from the Arduino at 50 Hz.
        self.create_timer(0.02, self._poll_rx)
        self.get_logger().info('cmd_vel_to_arduino node started')

    def _cmd_vel_callback(self, msg: Twist):
        floats = [
            msg.linear.x, msg.linear.y, msg.linear.z,
            msg.angular.x, msg.angular.y, msg.angular.z,
        ]
        send_size = self._link.tx_obj(floats)
        self._link.send(send_size)

    def _poll_rx(self):
        bytes_received = self._link.available()
        if bytes_received <= 0:
            return
        # Packet ID 0: plain string message from the Arduino, forwarded to the ROS logger.
        if self._link.id_byte == 0:
            text = self._link.rx_obj(obj_type=str, obj_byte_size=bytes_received)
            self.get_logger().info(f'Arduino: {text.rstrip()}')
        # Packet ID 1 (optional): encoder ticks as two int32 (left, right).
        elif self._link.id_byte == 1:
            left = self._link.rx_obj(obj_type=int, obj_byte_size=4, start_pos=0)
            right = self._link.rx_obj(obj_type=int, obj_byte_size=4, start_pos=4)
            self.get_logger().debug(f'Encoder ticks — left: {left}  right: {right}')
            # You could convert this to an Odometry message

    def destroy_node(self):
        self._link.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToArduino()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
