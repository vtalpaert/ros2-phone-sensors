import collections
import math
import struct
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, UInt8MultiArray
from pySerialTransfer import pySerialTransfer as txfer


class ROSStream:
    """Duck-type replacement for serial.Serial, backed by ROS usb/tx and usb/rx topics.

    SerialTransfer accesses: is_open, open(), close(), write(data), read(size), in_waiting.
    """

    def __init__(self, node: Node):
        self._logger = node.get_logger()
        self._pub = node.create_publisher(UInt8MultiArray, 'usb/tx', 10)
        node.create_subscription(UInt8MultiArray, 'usb/rx', self._rx_callback, 10)
        self._rx_buffer = collections.deque()
        self._lock = threading.Lock()
        self.is_open = False
        self._first_rx_logged = False

    def _rx_callback(self, msg: UInt8MultiArray):
        with self._lock:
            self._rx_buffer.extend(msg.data)
            if not self._first_rx_logged:
                self._first_rx_logged = True
                self._logger.info(f'first usb/rx packet received ({len(msg.data)} bytes)')

    def open(self):
        self.is_open = True

    def close(self):
        self.is_open = False

    def write(self, data: bytes) -> int:
        self._pub.publish(UInt8MultiArray(data=list(data)))
        return len(data)

    def read(self, size: int = 1) -> bytes:
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

        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value

        self._stream = ROSStream(self)

        # Bypass SerialTransfer's serial.Serial construction:
        # restrict_ports=False skips port validation, then we replace
        # link.connection with our ROS-backed stream before open() is called.
        self._link = txfer.SerialTransfer('', restrict_ports=False)
        self._link.connection = self._stream
        self._link.open()

        self.create_subscription(TwistStamped, 'cmd_vel', self._cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, 'arduino/odometry', 10)
        self.battery_pub = self.create_publisher(Float32, 'arduino/battery_voltage', 10)

        # Poll for incoming packets from the Arduino at 50 Hz.
        self.create_timer(0.02, self._poll_rx)
        self.get_logger().info('cmd_vel_to_arduino node started')

    def _cmd_vel_callback(self, msg: TwistStamped):
        twist = msg.twist
        floats = [
            twist.linear.x, twist.linear.y, twist.linear.z,
            twist.angular.x, twist.angular.y, twist.angular.z,
        ]
        send_size = self._link.tx_obj(floats)
        self._link.send(send_size)

    def _poll_rx(self):
        bytes_received = self._link.available()
        # status: NEW_DATA=2, NO_DATA=1, CONTINUE=3, CRC_ERROR=0, PAYLOAD_ERROR=-1, STOP_BYTE_ERROR=-2
        status = self._link.status
        if status in (-1, -2):
            names = {-1: 'PAYLOAD_ERROR', -2: 'STOP_BYTE_ERROR'}
            self.get_logger().warn(f'SerialTransfer parse error: {names[status]}')
            return
        if bytes_received <= 0:
            return
        packet_id = self._link.id_byte
        # Packet ID 0: plain string message from the Arduino, forwarded to the ROS logger.
        if packet_id == 0:
            text = self._link.rx_obj(obj_type=str, obj_byte_size=bytes_received)
            text = text.rstrip()
            if text.startswith('ERROR'):
                self.get_logger().error(f'Arduino: {text}')
            else:
                self.get_logger().info(f'Arduino: {text}')
        # Packet ID 1: odometry (x, y, theta, vx, omega) as five float32, SI units.
        elif packet_id == 1:
            payload = bytes(self._link.rx_buff[:20])
            x, y, theta, vx, omega = struct.unpack('<5f', payload)
            self._publish_odom(x, y, theta, vx, omega)
        # Packet ID 2: battery voltage as one float32.
        elif packet_id == 2:
            payload = bytes(self._link.rx_buff[:4])
            (voltage,) = struct.unpack('<f', payload)
            self.battery_pub.publish(Float32(data=voltage))
        else:
            self.get_logger().warn(f'unknown packet id {packet_id} ({bytes_received} bytes)')

    def _publish_odom(self, x, y, theta, vx, omega):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        msg.pose.pose.orientation.w = math.cos(theta / 2.0)
        msg.twist.twist.linear.x = vx
        msg.twist.twist.angular.z = omega
        self.odom_pub.publish(msg)

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
