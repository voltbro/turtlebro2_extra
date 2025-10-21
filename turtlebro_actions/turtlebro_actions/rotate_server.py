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
import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from tf_transformations import euler_from_quaternion

from turtlebro_interfaces.action import Rotation

from .motion_profile import compute_trapezoidal_speed, normalize_angle


class RotateServer(Node):
    """Action-сервер, который поворачивает робота вокруг оси Z."""

    def __init__(self) -> None:
        super().__init__('rotate_server_node')
        self.cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom = Odometry()
        self.create_subscription(Odometry, '/odom', self.subscriber_odometry_cb, 10)

        self._action_server = ActionServer(
            self,
            Rotation,
            'action_rotate',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )
        self.get_logger().info('Запущен Action-сервер поворота')

    def subscriber_odometry_cb(self, msg: Odometry) -> None:
        self.odom = msg

    def goal_callback(self, goal_request: Rotation.Goal) -> GoalResponse:
        self.get_logger().info(
            f'Получена цель поворота: угол={goal_request.goal} скорость={goal_request.speed:.3f}'
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle) -> CancelResponse:
        self.get_logger().info('Получен запрос на отмену поворота')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        goal = goal_handle.request
        total_angle_deg = abs(goal.goal)
        if total_angle_deg == 0:
            goal_handle.succeed()
            result = Rotation.Result()
            result.result = 0
            return result

        total_angle_rad = math.radians(total_angle_deg)
        direction = 1.0 if goal.goal >= 0 else -1.0
        max_speed = abs(goal.speed) if goal.speed != 0.0 else 0.9
        min_speed = min(max(max_speed * 0.25, 0.1), max_speed)

        accumulated_radians = 0.0
        prev_yaw = self._current_yaw()
        cmd = Twist()
        feedback = Rotation.Feedback()
        rate = self.create_rate(40)

        self.get_logger().info('Выполняю цель поворота')

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Цель поворота отменена')
                self.cmd_vel.publish(Twist())
                goal_handle.canceled()
                return self._build_result(accumulated_radians * direction)

            current_yaw = self._current_yaw()
            delta = normalize_angle(current_yaw - prev_yaw)
            prev_yaw = current_yaw
            accumulated_radians += delta * direction
            accumulated_radians = max(min(accumulated_radians, total_angle_rad), 0.0)

            feedback.feedback = int(round(math.degrees(accumulated_radians * direction)))
            goal_handle.publish_feedback(feedback)

            if abs(accumulated_radians) >= total_angle_rad - math.radians(0.5):
                self.cmd_vel.publish(Twist())
                goal_handle.succeed()
                self.get_logger().info('Цель поворота выполнена')
                return self._build_result(accumulated_radians * direction)

            speed = compute_trapezoidal_speed(
                abs(accumulated_radians),
                total_angle_rad,
                max_speed,
                min_speed=min_speed,
            )
            cmd.angular.z = direction * speed
            self.cmd_vel.publish(cmd)

            await rate.sleep()

        self.cmd_vel.publish(Twist())
        goal_handle.abort()
        return self._build_result(0.0)

    def _current_yaw(self) -> float:
        orientation = self.odom.pose.pose.orientation
        _, _, yaw = euler_from_quaternion(
            [
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            ]
        )
        return yaw

    def _build_result(self, radians_complete: float) -> Rotation.Result:
        result = Rotation.Result()
        result.result = int(round(math.degrees(radians_complete)))
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RotateServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Сервер поворота остановлен пользователем')
    finally:
        node.cmd_vel.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
