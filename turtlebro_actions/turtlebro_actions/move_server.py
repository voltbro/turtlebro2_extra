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

from turtlebro_interfaces.action import Move

from .motion_profile import compute_trapezoidal_speed


class MoveServer(Node):
    """Action-сервер, который перемещает робота вперед или назад."""

    def __init__(self) -> None:
        super().__init__('move_server_node')
        self.cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom = Odometry()
        self.create_subscription(Odometry, '/odom', self.subscriber_odometry_cb, 10)

        self._action_server = ActionServer(
            self,
            Move,
            'action_move',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )
        self.get_logger().info('Запущен Action-сервер перемещения')

    def subscriber_odometry_cb(self, msg: Odometry) -> None:
        self.odom = msg

    def goal_callback(self, goal_request: Move.Goal) -> GoalResponse:
        self.get_logger().info(
            f'Получена цель перемещения: расстояние={goal_request.goal:.3f}, '
            f'скорость={goal_request.speed:.3f}'
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle) -> CancelResponse:
        self.get_logger().info('Получен запрос на отмену перемещения')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        goal = goal_handle.request
        total_distance = float(abs(goal.goal))
        if math.isclose(total_distance, 0.0, abs_tol=1e-4):
            goal_handle.succeed()
            result = Move.Result()
            result.result = 0.0
            return result

        start_pose_x = float(self.odom.pose.pose.position.x)
        start_pose_y = float(self.odom.pose.pose.position.y)
        direction = 1.0 if goal.goal >= 0.0 else -1.0

        requested_speed = abs(goal.speed)
        max_speed = requested_speed if requested_speed > 0.0 else 0.09
        min_speed = min(max(max_speed * 0.2, 0.02), max_speed)

        cmd = Twist()
        feedback = Move.Feedback()
        rate = self.create_rate(40)

        self.get_logger().info('Выполняю цель перемещения')

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Цель перемещения отменена')
                self.cmd_vel.publish(Twist())
                goal_handle.canceled()
                distance_travelled = self._compute_distance(
                    start_pose_x, start_pose_y, self.odom.pose.pose.position
                )
                return self._build_result(distance_travelled, direction)

            distance_passed = self._compute_distance(
                start_pose_x, start_pose_y, self.odom.pose.pose.position
            )
            distance_passed = min(distance_passed, total_distance)

            feedback.feedback = float(direction * distance_passed)
            goal_handle.publish_feedback(feedback)

            if distance_passed >= total_distance - 1e-4:
                self.cmd_vel.publish(Twist())
                goal_handle.succeed()
                self.get_logger().info('Цель перемещения выполнена')
                return self._build_result(distance_passed, direction)

            speed = compute_trapezoidal_speed(
                distance_passed,
                total_distance,
                max_speed,
                min_speed=min_speed,
            )
            cmd.linear.x = direction * speed
            self.cmd_vel.publish(cmd)

            await rate.sleep()

        # Fallback in case rclpy shuts down
        self.cmd_vel.publish(Twist())
        goal_handle.abort()
        return self._build_result(0.0, direction)

    def _compute_distance(self, start_x: float, start_y: float, current_pose) -> float:
        dx = start_x - float(current_pose.x)
        dy = start_y - float(current_pose.y)
        return math.sqrt(dx * dx + dy * dy)

    def _build_result(self, distance: float, direction: float) -> Move.Result:
        result = Move.Result()
        result.result = float(direction * distance)
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MoveServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Сервер перемещения остановлен пользователем')
    finally:
        node.cmd_vel.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
