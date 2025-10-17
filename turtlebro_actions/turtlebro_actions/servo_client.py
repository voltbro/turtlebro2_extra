#! /usr/bin/env python3
import sys
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16


class ServoClient:
    """Helper around a std_msgs/UInt16 topic that controls a servo."""

    def __init__(self, node: Node, servo: int) -> None:
        self._node = node
        self.servo = servo
        topic = f'/servo{servo}'
        self.pub = node.create_publisher(UInt16, topic, 10)
        self._node.get_logger().info('Servo client ready on %s', topic)

    def move_await(self, angle: int) -> None:
        self._node.get_logger().info('Move servo %s to %i (await)', self.servo, angle)
        while rclpy.ok() and self.pub.get_subscription_count() < 1:
            time.sleep(0.2)
        self.move(angle)

    def move(self, angle: int) -> None:
        self._node.get_logger().info('Move servo %s to %i', self.servo, angle)
        msg = UInt16()
        msg.data = int(angle)
        self.pub.publish(msg)
        time.sleep(1.0)

    def shutdown(self) -> None:
        self._node.get_logger().info('Servo %s client shutdown', self.servo)
        self._node.destroy_publisher(self.pub)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = Node('servo_client_node')
    try:
        servo_num = 44
        if len(sys.argv) > 1:
            servo_num = int(sys.argv[1])
        servo_client = ServoClient(node, servo_num)
        servo_client.move_await(90)
    except Exception as exc:  # noqa: BLE001
        node.get_logger().error('Servo client failed: %s', exc)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
