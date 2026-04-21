# Copyright 2024 VoltBro
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
#
# See the License for the specific language governing permissions and
# limitations under the License.
#
from __future__ import annotations

from dataclasses import dataclass
import math

from .motion_profile import normalize_angle


@dataclass(frozen=True)
class LinearPidGains:
    """Коэффициенты контура «ошибка yaw → угловая скорость ω».

    В ROS задаются с префиксом, например: ``linear_pid.kp``, ``linear_pid.ki``.
    """

    kp: float
    ki: float
    kd: float
    # Макс. |накопленная сумма для I| (рад·с); <= 0 — без ограничения интеграла
    integral_limit: float = 0.0


class LinearPidController:
    """Дискретный PID: на входе ошибка по yaw (рад), на выходе ω (рад/с) для ``Twist.angular.z``."""

    __slots__ = ('_gains', '_output_limit', '_i_accum', '_e_prev', '_first')

    def __init__(self, gains: LinearPidGains, *, output_limit: float) -> None:
        self._gains = gains
        self._output_limit = max(0.0, output_limit)
        self._i_accum = 0.0
        self._e_prev = 0.0
        self._first = True

    def reset(self) -> None:
        self._i_accum = 0.0
        self._e_prev = 0.0
        self._first = True

    def step(self, yaw_error_rad: float, dt: float) -> float:
        """yaw_error_rad — нормализованная ошибка [-π, π]. Возвращает ω (рад/с)."""
        e = yaw_error_rad
        dt = max(dt, 1e-6)

        if self._first:
            self._e_prev = e
            self._first = False
            de_dt = 0.0
        else:
            de_dt = normalize_angle(e - self._e_prev) / dt

        self._i_accum += e * dt
        ilim = self._gains.integral_limit
        if ilim > 0.0:
            self._i_accum = max(-ilim, min(ilim, self._i_accum))

        u = self._gains.kp * e + self._gains.ki * self._i_accum + self._gains.kd * de_dt
        self._e_prev = e

        lim = self._output_limit
        if lim > 0.0:
            u = max(-lim, min(lim, u))
        elif not math.isfinite(u):
            u = 0.0
        return u
