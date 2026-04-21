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
from pathlib import Path
import math
import sys

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from turtlebro_actions.utils.motion_profile import (  # noqa: E402
    normalize_angle,
    rotation_progress_from_start_rad,
)


def test_rotation_progress_ccw_quarter_turn():
    start = 0.0
    total = math.pi / 2.0
    p = rotation_progress_from_start_rad(start, math.pi / 4.0, 1.0, total, 0.0)
    assert abs(p - math.pi / 4.0) < 1e-9
    p2 = rotation_progress_from_start_rad(start, math.pi / 2.0, 1.0, total, p)
    assert abs(p2 - total) < 1e-9


def test_rotation_progress_overshoot_not_clamped_to_goal():
    start = 0.0
    total = math.pi / 2.0
    overshoot = 1.6554076671600342
    p = rotation_progress_from_start_rad(start, overshoot, 1.0, total, 0.0)
    assert abs(p - overshoot) < 1e-9
    assert p > total


def test_rotation_progress_cw():
    start = 0.0
    total = math.radians(60.0)
    cur = normalize_angle(-math.radians(30.0))
    p = rotation_progress_from_start_rad(start, cur, -1.0, total, 0.0)
    assert abs(p - math.radians(30.0)) < 1e-9


def test_rotation_progress_monotonic():
    start = 0.0
    total = math.radians(45.0)
    p0 = rotation_progress_from_start_rad(start, math.radians(20.0), 1.0, total, 0.0)
    p1 = rotation_progress_from_start_rad(start, math.radians(10.0), 1.0, total, p0)
    assert p1 >= p0


def test_rotation_progress_wrap_180_goal():
    start = 0.0
    total = math.pi
    # rel = -pi даёт signed = -pi; коррекция +2pi
    p = rotation_progress_from_start_rad(start, -math.pi, 1.0, total, 0.0)
    assert abs(p - math.pi) < 1e-9


def test_incremental_yaw_sum_matches_true_rotation():
    """Ветка >pi: сумма normalize(odom - prev) восстанавливает полный угол при обёртке [-pi, pi]."""
    true_yaw = 0.0
    prev_odom = normalize_angle(true_yaw)
    cum = 0.0
    step = math.radians(11.0)
    n = int(math.ceil(math.radians(450) / step))
    for _ in range(n):
        true_yaw += step
        y_odom = normalize_angle(true_yaw)
        cum += normalize_angle(y_odom - prev_odom)
        prev_odom = y_odom
    assert abs(cum - true_yaw) < 1e-6
    assert cum >= math.radians(450) - step
