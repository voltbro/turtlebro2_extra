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
"""Расширение TurtleBro с поддержкой навигации Nav2."""

from __future__ import annotations

import math

import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from tf_transformations import quaternion_from_euler

from .turtlebro_py import TurtleBro


class TurtleBroNav(TurtleBro):
    """Робот с автономной навигацией через Nav2 (navigate_to_pose)."""

    def __init__(self) -> None:
        super().__init__()
        self._nav_client = ActionClient(self._node, NavigateToPose, 'navigate_to_pose')

    def goto(self, x: float, y: float, theta: float = 0) -> None:
        """Отправить цель навигации в плоскости карты."""
        self.__goto(x, y, theta)

    def __goal_message_assemble(self, x: float, y: float, theta: float) -> NavigateToPose.Goal:
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self._node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)

        q = quaternion_from_euler(0, 0, math.radians(float(theta)))
        goal.pose.pose.orientation.x = q[0]
        goal.pose.pose.orientation.y = q[1]
        goal.pose.pose.orientation.z = q[2]
        goal.pose.pose.orientation.w = q[3]

        return goal

    def __goto(self, x: float, y: float, theta: float) -> None:
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError('Action-сервер NavigateToPose недоступен')

        goal = self.__goal_message_assemble(x, y, theta)
        send_future = self._nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self._node, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError('Цель NavigateToPose отклонена')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self._node, result_future)
