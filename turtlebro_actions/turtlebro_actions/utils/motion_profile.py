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
from __future__ import annotations

import math


def compute_trapezoidal_speed(
    distance_passed: float,
    total_distance: float,
    max_speed: float,
    *,
    min_speed: float,
    ramp_time_sec: float = 2.0,
) -> float:
    """Возвращает скорость для трапецеидального профиля по пройденному пути.

    Длина участка разгона/торможения рассчитывается из желаемого времени
    ``ramp_time_sec`` по формуле ``0.5 * (v_min + v_max) * T_ramp`` (точная для
    distance-linear профиля со средней скоростью ``(v_min+v_max)/2``).

    Если общий путь короче, чем сумма полноценного разгона и торможения
    (``total_distance < (v_min + v_max) * T_ramp``), срабатывает клэмп
    ``ramp_distance <= total_distance / 2`` и профиль становится треугольным:
    робот не успевает выйти на ``max_speed``.
    """
    total_distance = max(total_distance, 0.0)
    if total_distance == 0.0:
        return 0.0

    max_speed = max(max_speed, 0.0)
    min_speed = max(min_speed, 0.0)
    if max_speed == 0.0:
        return 0.0

    ramp_time_sec = max(ramp_time_sec, 0.0)
    ramp_distance = max(0.5 * (min_speed + max_speed) * ramp_time_sec, 0.01)
    ramp_distance = min(ramp_distance, total_distance / 2.0)

    distance_passed = max(min(distance_passed, total_distance), 0.0)
    distance_remaining = max(total_distance - distance_passed, 0.0)

    if ramp_distance == 0.0:
        target_speed = max_speed
    elif distance_passed < ramp_distance:
        ratio = distance_passed / ramp_distance
        target_speed = min_speed + (max_speed - min_speed) * ratio
    elif distance_remaining < ramp_distance:
        ratio = distance_remaining / ramp_distance
        target_speed = min_speed + (max_speed - min_speed) * ratio
    else:
        target_speed = max_speed

    return max(min(target_speed, max_speed), min_speed)


def normalize_angle(angle: float) -> float:
    """Нормализует угол в диапазон [-pi, pi]."""
    if math.isfinite(angle):
        angle = (angle + math.pi) % (2.0 * math.pi) - math.pi
    else:
        angle = 0.0
    return angle


def rotation_progress_from_start_rad(
    start_yaw: float,
    current_yaw: float,
    direction: float,
    total_angle_rad: float,
    prev_progress_rad: float,
) -> float:
    """Прогресс (рад) для целей < π: разность yaw от старта, с монотонностью по ``prev``."""
    total_angle_rad = max(total_angle_rad, 0.0)
    rel = normalize_angle(current_yaw - start_yaw)
    signed = rel * direction
    if signed < 0.0 and total_angle_rad >= math.pi - 1e-9:
        signed += 2.0 * math.pi
    candidate = max(0.0, signed)
    return max(candidate, max(0.0, prev_progress_rad))
