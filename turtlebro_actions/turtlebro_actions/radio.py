
#! /usr/bin/env python3
import struct
import time
from typing import Optional

import rclpy
import serial
from rclpy.node import Node

from .commands_controller import CommandsController

TERM = b'\x00\xFF\xFF'
STRUCT_FORMAT = '!BH'
PACK_SIZE = 1024


class RadioCommandNode(Node):
    """Bridges serial packets into action/service calls."""

    def __init__(self) -> None:
        super().__init__('radio_command_node')
        self.declare_parameter(
            'port',
            '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
        )
        self.declare_parameter('baud', 19200)
        self.declare_parameter('linear_speed', 0.22)
        self.declare_parameter('angular_speed', 1.0)

        self.serial = serial.Serial()
        self.serial.port = self.get_parameter('port').get_parameter_value().string_value
        self.serial.baudrate = self.get_parameter('baud').get_parameter_value().integer_value
        self.serial.dtr = False
        self.serial.timeout = 0.5
        self.serial.open()

        linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value
        angular_speed = self.get_parameter('angular_speed').get_parameter_value().double_value

        self.controller = CommandsController(
            self,
            linear_speed=float(linear_speed),
            angular_speed=float(angular_speed),
        )
        self.get_logger().info(
            'Radio command node ready on %s @ %s baud', self.serial.port, self.serial.baudrate
        )

    def spin_forever(self) -> None:
        struct_len = struct.calcsize(STRUCT_FORMAT)
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.0)
                pack = self.serial.read_until(TERM)
                if len(pack) <= 2 or not pack.endswith(TERM):
                    continue

                payload = pack[:-len(TERM)]
                commands_count = int(len(payload) / struct_len)

                for idx in range(commands_count):
                    try:
                        command, value = struct.unpack(
                            STRUCT_FORMAT, payload[idx * struct_len : (idx + 1) * struct_len]
                        )
                        self._process_command(idx, command, value)
                    except struct.error as exc:
                        self.get_logger().warn('Malformed packet: %s', exc)
        except serial.SerialException as exc:
            self.get_logger().error('Serial failure: %s', exc)
        finally:
            self.serial.close()

    def _process_command(self, idx: int, command: int, value: int) -> None:
        self.get_logger().debug('Packet %d command %d value %d', idx, command, value)
        result = self.controller.execute(command, value)

        if command in (55, 56) and result is not None:
            data: bytes = result if isinstance(result, bytes) else result.encode()
            packs = int(len(data) / PACK_SIZE) + 1
            for pack_idx in range(packs):
                serial_pack = data[pack_idx * PACK_SIZE : (pack_idx + 1) * PACK_SIZE]
                sent = self.serial.write(serial_pack)
                self.get_logger().info('Send data pack %d: %s bytes', pack_idx, sent)
                time.sleep(0.5)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = RadioCommandNode()
    try:
        node.spin_forever()
    except KeyboardInterrupt:
        node.get_logger().info('Radio command node interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
