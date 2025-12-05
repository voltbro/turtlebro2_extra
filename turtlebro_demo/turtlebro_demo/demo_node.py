#!/usr/bin/env python3
#
# Copyright 2025 VoltBro
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
"""Наглядный демонстрационный сценарий возможностей turtlebro_py."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Callable, List, Optional

import rclpy
from rclpy.node import Node

from turtlebro_py import TurtleBro

LED_STRIP_LEN = 24

AnnouncementBuilder = Callable[['ScenarioRunner'], str]
ActionCallback = Callable[['ScenarioRunner'], None]


@dataclass
class DemoSettings:
    """Параметры сценария, считываемые из ROS-параметров."""

    forward_distance_m: float
    backward_distance_m: float
    rotation_angle_deg: float
    voice: str
    speech_rate: int
    speech_language: str
    speech_punctuation_mode: str
    pause_between_steps_s: float
    led_intro_color: str
    led_finish_color: str
    photo_filename: str
    distance_angle_deg: int
    enable_led_show: bool
    enable_distance_report: bool
    enable_photo: bool
    enable_audio: bool
    record_duration_s: float
    audio_device: str
    record_filename: str

    @staticmethod
    def from_node(node: Node) -> 'DemoSettings':
        def _float_param(name: str, minimum: float = 0.0) -> float:
            value = float(node.get_parameter(name).value)
            clamped = value if value >= minimum else minimum
            if clamped != value:
                node.get_logger().warning(
                    'Параметр %s скорректирован до %.2f (минимум %.2f)', name, clamped, minimum
                )
            return clamped

        def _int_param(name: str, minimum: int = 0) -> int:
            value = int(math.floor(float(node.get_parameter(name).value)))
            if value < minimum:
                node.get_logger().warning(
                    'Параметр %s скорректирован до %d (минимум %d)', name, minimum, minimum
                )
                return minimum
            return value

        def _str_param(name: str) -> str:
            return str(node.get_parameter(name).value).strip()

        return DemoSettings(
            forward_distance_m=_float_param('forward_distance_m'),
            backward_distance_m=_float_param('backward_distance_m'),
            rotation_angle_deg=_float_param('rotation_angle_deg'),
            voice=_str_param('voice'),
            speech_rate=_int_param('speech_rate'),
            speech_language=_str_param('speech_language') or 'ru',
            speech_punctuation_mode=_str_param('speech_punctuation_mode') or 'SOME',
            pause_between_steps_s=_float_param('pause_between_steps_s'),
            led_intro_color=_str_param('led_intro_color') or 'yellow',
            led_finish_color=_str_param('led_finish_color') or 'blue',
            photo_filename=_str_param('photo_filename') or 'demo_photo',
            distance_angle_deg=min(359, max(0, _int_param('distance_angle_deg', minimum=0))),
            enable_led_show=bool(node.get_parameter('enable_led_show').value),
            enable_distance_report=bool(node.get_parameter('enable_distance_report').value),
            enable_photo=bool(node.get_parameter('enable_photo').value),
            enable_audio=bool(node.get_parameter('enable_audio').value),
            record_duration_s=_float_param('record_duration_s', minimum=0.1),
            audio_device=_str_param('audio_device'),
            record_filename=_str_param('record_filename') or 'demo_recording',
        )


@dataclass
class ScenarioState:
    """Состояния, которые накапливаются между шагами сценария."""

    recorded_audio_path: Optional[str] = None
    last_photo_path: Optional[str] = None
    last_distance_m: Optional[float] = None


@dataclass
class ScenarioStep:
    """Отдельное действие демонстратора."""

    name: str
    announcement: AnnouncementBuilder
    action: ActionCallback
    pause_override: Optional[float] = None


class ScenarioRunner:
    """Управляет последовательным выполнением шагов и озвучиванием."""

    def __init__(self, node: Node, robot: TurtleBro, settings: DemoSettings) -> None:
        self._node = node
        self.robot = robot
        self.settings = settings
        self.state = ScenarioState()

    def speak(self, text: str) -> None:
        if not text:
            return
        try:
            self.robot.say(
                text,
                voice=self.settings.voice,
                rate=self.settings.speech_rate,
                language=self.settings.speech_language,
                punctuation_mode=self.settings.speech_punctuation_mode,
            )
        except Exception as exc:  # noqa: BLE001
            self._node.get_logger().warning(f'Не получилось озвучить фразу "{text}": {exc}')

    def pause(self, override: Optional[float]) -> None:
        duration = self.settings.pause_between_steps_s if override is None else max(0.0, override)
        if duration > 0:
            time.sleep(duration)

    def run(self, steps: List[ScenarioStep]) -> None:
        for index, step in enumerate(steps, start=1):
            self._node.get_logger().info(f'Шаг {index}/{len(steps)}: {step.name}')
            announcement = step.announcement(self)
            if announcement:
                self.speak(announcement)
            try:
                step.action(self)
            except KeyboardInterrupt:
                raise
            except Exception as exc:  # noqa: BLE001
                self._node.get_logger().error(f'Шаг "{step.name}" завершился ошибкой: {exc}')
            self.pause(step.pause_override)

    def set_head_led(self, color: str) -> None:
        try:
            self.robot.color(color)
        except Exception as exc:  # noqa: BLE001
            self._node.get_logger().warning(
                f'Не удалось установить цвет светодиода "{color}": {exc}'
            )

    def set_backlight_all(self, color: str) -> None:
        try:
            self.robot.backlight_all(color)
        except Exception as exc:  # noqa: BLE001
            self._node.get_logger().warning(f'Не удалось обновить подсветку: {exc}')

    def set_backlight_gradient(self) -> None:
        gradient = []
        denominator = max(LED_STRIP_LEN - 1, 1)
        for idx in range(LED_STRIP_LEN):
            phase = idx / denominator
            gradient.append((phase, 1.0 - phase * 0.5, 1.0 - phase, 1.0))
        try:
            self.robot.backlight_array(gradient)
        except Exception as exc:  # noqa: BLE001
            self._node.get_logger().warning(f'Не удалось выставить градиент подсветки: {exc}')


class TurtleBroDemoNode(Node):
    """Основной узел, подготавливающий TurtleBro и запускающий демонстрацию."""

    def __init__(self) -> None:
        super().__init__('turtlebro_demo')
        self._declare_parameters()
        self._settings = DemoSettings.from_node(self)
        self._robot = TurtleBro(node=self)
        self._runner = ScenarioRunner(self, self._robot, self._settings)

    def _declare_parameters(self) -> None:
        self.declare_parameter('forward_distance_m', 0.5)
        self.declare_parameter('backward_distance_m', 0.3)
        self.declare_parameter('rotation_angle_deg', 90.0)
        self.declare_parameter('voice', 'Vsevolod')
        self.declare_parameter('speech_rate', 15)
        self.declare_parameter('speech_language', 'ru')
        self.declare_parameter('speech_punctuation_mode', 'SOME')
        self.declare_parameter('pause_between_steps_s', 2.0)
        self.declare_parameter('led_intro_color', 'yellow')
        self.declare_parameter('led_finish_color', 'blue')
        self.declare_parameter('photo_filename', 'demo_photo')
        self.declare_parameter('distance_angle_deg', 0)
        self.declare_parameter('enable_led_show', True)
        self.declare_parameter('enable_distance_report', True)
        self.declare_parameter('enable_photo', True)
        self.declare_parameter('enable_audio', True)
        self.declare_parameter('record_duration_s', 3.0)
        self.declare_parameter('audio_device', '')
        self.declare_parameter('record_filename', 'demo_recording')

    def run_demo(self) -> None:
        steps = self._build_steps()
        self._runner.run(steps)

    def shutdown(self) -> None:
        try:
            self._robot.close()
        finally:
            self.destroy_node()

    def _build_steps(self) -> List[ScenarioStep]:
        settings = self._settings
        steps: List[ScenarioStep] = []

        steps.append(
            ScenarioStep(
                name='Вступление',
                announcement=lambda _: (
                    'Привет! Я TurtleBro. Сейчас покажу, что умею: свет, движение, сенсоры и звук.'
                ),
                action=self._intro_action,
            )
        )

        if settings.enable_led_show:
            steps.append(
                ScenarioStep(
                    name='Световое шоу',
                    announcement=lambda _: 'Меняю подсветку, чтобы показать управление светом.',
                    action=self._led_show_action,
                )
            )

        if settings.enable_distance_report:
            steps.append(
                ScenarioStep(
                    name='Лазерный дальномер',
                    announcement=lambda runner: (
                        f'Проверяю расстояние на угле {runner.settings.distance_angle_deg} градусов.'
                    ),
                    action=self._distance_action,
                )
            )

        if settings.forward_distance_m > 0.0:
            steps.append(
                ScenarioStep(
                    name='Движение вперед',
                    announcement=lambda runner: (
                        f'Еду вперед на {runner.settings.forward_distance_m:.2f} метра.'
                    ),
                    action=self._forward_action,
                )
            )

        if settings.backward_distance_m > 0.0:
            steps.append(
                ScenarioStep(
                    name='Движение назад',
                    announcement=lambda runner: (
                        f'Возвращаюсь назад на {runner.settings.backward_distance_m:.2f} метра.'
                    ),
                    action=self._backward_action,
                )
            )

        if settings.rotation_angle_deg > 0.0:
            steps.append(
                ScenarioStep(
                    name='Поворот',
                    announcement=lambda runner: (
                        f'Разворачиваюсь на {runner.settings.rotation_angle_deg:.0f} градусов.'
                    ),
                    action=self._rotation_action,
                )
            )

        if settings.enable_photo:
            steps.append(
                ScenarioStep(
                    name='Фотоснимок',
                    announcement=lambda runner: (
                        f'Делаю снимок и сохраняю как {runner.settings.photo_filename}.jpg.'
                    ),
                    action=self._photo_action,
                )
            )

        if settings.enable_audio:
            steps.append(
                ScenarioStep(
                    name='Запись аудио',
                    announcement=lambda runner: (
                        f'Записываю звук на {runner.settings.record_duration_s:.1f} секунды.'
                    ),
                    action=self._record_audio_action,
                )
            )
            steps.append(
                ScenarioStep(
                    name='Воспроизведение аудио',
                    announcement=lambda _: 'Проигрываю свежую запись, чтобы вы меня услышали.',
                    action=self._play_audio_action,
                )
            )

        steps.append(
            ScenarioStep(
                name='Финал',
                announcement=lambda _: 'Демонстрация окончена, перехожу в спокойный режим.',
                action=self._finish_action,
            )
        )

        return steps

    # --- Шаги сценария ---

    def _intro_action(self, runner: ScenarioRunner) -> None:
        runner.set_head_led(runner.settings.led_intro_color)
        runner.set_backlight_all(runner.settings.led_intro_color)
        self.get_logger().info(
            f'Подсветка установлена в стартовый цвет {runner.settings.led_intro_color}'
        )

    def _led_show_action(self, runner: ScenarioRunner) -> None:
        runner.set_backlight_gradient()
        runner.set_head_led('white')
        self.get_logger().info('Градиент подсветки включен')

    def _distance_action(self, runner: ScenarioRunner) -> None:
        angle = runner.settings.distance_angle_deg
        distance = runner.robot.distance(angle)
        if distance is None or distance == float('inf'):
            runner.state.last_distance_m = None
            runner.speak('Данных от лидара нет, двигаюсь осторожно.')
            self.get_logger().warning(f'Не удалось получить расстояние по углу {angle}')
            return
        runner.state.last_distance_m = float(distance)
        runner.speak(f'Свободно примерно {runner.state.last_distance_m:.2f} метра.')
        self.get_logger().info(
            f'Расстояние по углу {angle} составляет {runner.state.last_distance_m:.2f} м'
        )

    def _forward_action(self, runner: ScenarioRunner) -> None:
        runner.robot.forward(runner.settings.forward_distance_m)
        self.get_logger().info(f'Проехал вперед {runner.settings.forward_distance_m:.2f} м')

    def _backward_action(self, runner: ScenarioRunner) -> None:
        runner.robot.backward(runner.settings.backward_distance_m)
        self.get_logger().info(f'Проехал назад {runner.settings.backward_distance_m:.2f} м')

    def _rotation_action(self, runner: ScenarioRunner) -> None:
        angle = runner.settings.rotation_angle_deg
        runner.robot.left(angle)
        self.get_logger().info(f'Повернулся на {angle:.0f}° влево')

    def _photo_action(self, runner: ScenarioRunner) -> None:
        filename = runner.settings.photo_filename
        runner.robot.save_photo(filename)
        runner.state.last_photo_path = f'/home/pi/{filename}.jpg'
        runner.speak('Фото готово, сохраняю файл на контроллере.')
        self.get_logger().info(f'Фото сохранено по пути {runner.state.last_photo_path}')

    def _record_audio_action(self, runner: ScenarioRunner) -> None:
        duration = runner.settings.record_duration_s
        filepath = runner.robot.record(
            timeval=duration,
            filename=runner.settings.record_filename,
            device=runner.settings.audio_device,
        )
        runner.state.recorded_audio_path = filepath
        self.get_logger().info(f'Аудиофайл записан: {filepath}')

    def _play_audio_action(self, runner: ScenarioRunner) -> None:
        target = runner.state.recorded_audio_path
        if not target:
            raise RuntimeError('Нет записанного аудио для воспроизведения')
        runner.robot.play(target, blocking=True, device=runner.settings.audio_device)
        self.get_logger().info(f'Воспроизведение файла {target} завершено')

    def _finish_action(self, runner: ScenarioRunner) -> None:
        runner.set_head_led(runner.settings.led_finish_color)
        runner.set_backlight_all(runner.settings.led_finish_color)
        self.get_logger().info(
            f'Подсветка переведена в финальный цвет {runner.settings.led_finish_color}'
        )


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = TurtleBroDemoNode()
    try:
        node.run_demo()
    except KeyboardInterrupt:
        node.get_logger().info('Демонстрация остановлена пользователем')
    finally:
        node.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
