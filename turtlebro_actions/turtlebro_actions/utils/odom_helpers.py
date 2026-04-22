# Copyright 2024 VoltBro
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
#
# See the License for the specific language governing permissions and
# limitations under the License.
#
"""Утилиты для работы с ``nav_msgs/Odometry``."""
from __future__ import annotations

import math

from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion


def current_yaw(odom: Odometry) -> float:
    """Возвращает yaw (рад) из ``odom.pose.pose.orientation``."""
    q = odom.pose.pose.orientation
    _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
    return yaw


def distance_xy(start_x: float, start_y: float, point) -> float:
    """Евклидово расстояние по XY от (``start_x``, ``start_y``) до ``point`` с полями ``x``/``y``."""
    dx = float(point.x) - start_x
    dy = float(point.y) - start_y
    return math.sqrt(dx * dx + dy * dy)
