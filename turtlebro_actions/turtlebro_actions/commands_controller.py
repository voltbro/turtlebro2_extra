#! /usr/bin/env python3
import struct
import time
from typing import Optional

import rclpy
from rclpy.node import Node

from .move_client import MoveClient
from .rotate_client import RotateClient
from .servo_client import ServoClient
from .video_client import VideoClient


class CommandsController:
    """Dispatches radio commands to the corresponding action or topic clients."""

    def __init__(
        self,
        node: Optional[Node] = None,
        linear_speed: float = 0.22,
        angular_speed: float = 1.0,
    ) -> None:
        self._owns_node = False
        if node is None:
            if not rclpy.is_initialized():
                rclpy.init()
            node = rclpy.create_node('commands_control_node')
            self._owns_node = True
        self._node = node
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed

        self._node.get_logger().info('Init clients')
        self._node.get_logger().info('Set linear speed %.3f', self.linear_speed)
        self._node.get_logger().info('Set angular speed %.3f', self.angular_speed)

        self.movie_client = MoveClient(node)
        self.rotate_client = RotateClient(node)
        self.video_client = VideoClient(node)
        self.servo44 = ServoClient(node, 44)
        self.servo45 = ServoClient(node, 45)

    def execute(self, command: int, value: int) -> Optional[object]:
        self._node.get_logger().info('Execute: %s, value: %s', command, value)

        if command == 1:
            self.linear_speed = value / 100.0
            self._node.get_logger().info('Set linear speed %.3f', self.linear_speed)

        elif command == 2:
            self.angular_speed = value / 1000.0
            self._node.get_logger().info('Set angular speed %.3f', self.angular_speed)

        elif command == 11:
            distance = value / 100.0
            return self.movie_client.SendGoal(distance, self.linear_speed)

        elif command == 12:
            distance = value / 100.0
            return self.movie_client.SendGoal(-distance, self.linear_speed)

        elif command == 15:
            return self.rotate_client.SendGoal(value, self.angular_speed)

        elif command == 16:
            return self.rotate_client.SendGoal(-value, self.angular_speed)

        elif command == 21:
            self.servo44.move(value)

        elif command == 22:
            self.servo45.move(value)

        elif command == 41:
            time.sleep(value / 1000.0)

        elif command == 55:
            return self.video_client.getImage()

        elif command == 56:
            return '1' * value

        return None

    def shutdown(self) -> None:
        if self._owns_node:
            self._node.destroy_node()
            rclpy.shutdown()


def main(args=None) -> None:
    if not rclpy.is_initialized():
        rclpy.init(args=args)
    node = rclpy.create_node('commands_control_node')
    controller = CommandsController(node)

    struct_format = '!BH'
    struct_len = struct.calcsize(struct_format)
    chunks = struct.pack(struct_format, 11, 30)
    chunks += struct.pack(struct_format, 12, 30)
    chunks += struct.pack(struct_format, 15, 30)
    chunks += struct.pack(struct_format, 16, 30)

    commands_count = int(len(chunks) / struct_len)

    for i in range(commands_count):
        try:
            command, value = struct.unpack(
                struct_format,
                chunks[i * struct_len : (i + 1) * struct_len],
            )
            node.get_logger().info('Test command %s value %s', command, value)
            controller.execute(command, value)
        except Exception as exc:  # noqa: BLE001
            node.get_logger().error('Command execution failed: %s', exc)

    controller.shutdown()

    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
