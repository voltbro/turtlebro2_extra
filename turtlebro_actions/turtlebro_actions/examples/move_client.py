#!/usr/bin/env python3

# Copyright 2024 VoltBro
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
import sys
from typing import Optional

import rclpy
from rclpy.action.client import ActionClient
from rclpy.node import Node

from turtlebro_interfaces.action import Move


class MoveClient:
    """Простой клиент Action перемещения, сохраняющий прежний API."""

    def __init__(self, node: Node, wait_timeout: float = 5.0) -> None:
        self._node = node
        self._client = ActionClient(node, Move, 'action_move')
        self._wait_for_server(wait_timeout)

    def _wait_for_server(self, timeout: float) -> None:
        if not self._client.wait_for_server(timeout_sec=timeout):
            raise RuntimeError('Action-сервер перемещения недоступен')
        self._node.get_logger().info('Клиент перемещения подключен')

    def SendGoal(self, goal_val: float, speed_val: float) -> Move.Result:
        goal = Move.Goal()
        goal.goal = float(goal_val)
        goal.speed = float(speed_val)

        send_goal_future = self._client.send_goal_async(
            goal,
            feedback_callback=self._action_feedback,
        )
        rclpy.spin_until_future_complete(self._node, send_goal_future)
        goal_handle = send_goal_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self._node.get_logger().warn('Цель перемещения отклонена')
            result = Move.Result()
            result.result = 0.0
            return result

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self._node, result_future)
        goal_result = result_future.result()

        if goal_result is None:
            result = Move.Result()
            result.result = 0.0
            return result

        return goal_result.result

    def _action_feedback(self, feedback_msg: Move.Feedback) -> None:
        self._node.get_logger().info(f'Текущее смещение: {feedback_msg.feedback:.3f}')


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = Node('move_client_node')
    try:
        client = MoveClient(node)
        result = client.SendGoal(0.5, 0.22)
        node.get_logger().info(f'Результат действия перемещения: {result.result:.3f}')
    except Exception as exc:  # noqa: BLE001
        node.get_logger().error(f'Сбой клиента перемещения: {exc}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
