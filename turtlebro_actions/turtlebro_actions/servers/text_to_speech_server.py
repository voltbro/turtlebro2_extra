#!/usr/bin/env python3
#
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
"""Action-сервер синтеза речи на основе RHVoice (через speech-dispatcher)."""

from __future__ import annotations

import time
from typing import Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

try:
    import speechd
except ImportError:  # pragma: no cover - ожидание, что библиотека подключена на роботе
    speechd = None  # type: ignore[assignment]

from turtlebro_interfaces.action import TextToSpeech


class TextToSpeechServer(Node):
    """Action-сервер, который отправляет текст в RHVoice через speech-dispatcher."""

    def __init__(self) -> None:
        super().__init__('text_to_speech_server')
        self._speech_client: Optional['speechd.SSIPClient'] = None

        self._action_server = ActionServer(
            self,
            TextToSpeech,
            'text_to_speech',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )
        self.get_logger().info('Запущен Action-сервер синтеза речи')

    def destroy_node(self) -> None:
        self._close_speech_client()
        return super().destroy_node()

    def _close_speech_client(self) -> None:
        client = self._speech_client
        self._speech_client = None
        if client is not None:
            try:
                client.close()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning(f'Ошибка при закрытии клиента speech-dispatcher: {exc}')

    def _init_speech_client(self) -> None:
        if speechd is None:
            self.get_logger().error(
                'Модуль speechd недоступен. Установите python3-speechd и настройте RHVoice.'
            )
            return
        try:
            client = speechd.SSIPClient('turtlebro_tts')
            client.set_output_module('rhvoice')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Не удалось подключиться к speech-dispatcher: {exc}')
            return

        self._speech_client = client

    def _ensure_speech_client(self) -> None:
        if self._speech_client is None:
            self._init_speech_client()

    def goal_callback(self, goal_request: TextToSpeech.Goal) -> GoalResponse:
        self._ensure_speech_client()
        if self._speech_client is None:
            self.get_logger().error('Запрос синтеза отклонен: speech-dispatcher недоступен')
            return GoalResponse.REJECT
        if not goal_request.text:
            self.get_logger().warning('Запрос синтеза отклонен: текст пустой')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle) -> CancelResponse:
        self.get_logger().info('Получен запрос на отмену синтеза речи')
        self._stop_synthesis()
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        goal = goal_handle.request
        assert isinstance(goal, TextToSpeech.Goal)

        result = TextToSpeech.Result()
        feedback = TextToSpeech.Feedback()

        if self._speech_client is None:
            result.success = False
            result.message = 'speech-dispatcher недоступен'
            goal_handle.abort()
            return result

        try:
            self._apply_goal_settings(goal)
            feedback.state = 'queued'
            goal_handle.publish_feedback(feedback)

            self._speech_client.speak(goal.text)
            feedback.state = 'speaking'
            goal_handle.publish_feedback(feedback)

            completed = self._wait_until_done(goal_handle, text=goal.text)
            if goal_handle.is_cancel_requested:
                self._stop_synthesis()
                goal_handle.canceled()
                result.success = False
                result.message = 'Синтез отменен'
                return result

            if not completed:
                self.get_logger().debug('Таймаут ожидания окончания синтеза речи')

            feedback.state = 'completed'
            goal_handle.publish_feedback(feedback)
            goal_handle.succeed()
            result.success = True
            result.message = 'Синтез завершен'
            return result
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Ошибка синтеза речи: {exc}')
            self._stop_synthesis()
            self._close_speech_client()
            result.success = False
            result.message = str(exc)
            goal_handle.abort()
            return result

    def _apply_goal_settings(self, goal: TextToSpeech.Goal) -> None:
        if self._speech_client is None:
            return

        if goal.voice:
            try:
                self._speech_client.set_synthesis_voice(goal.voice)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning(f'Не удалось установить голос {goal.voice}: {exc}')
        if goal.rate:
            try:
                self._speech_client.set_rate(int(goal.rate))
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning(f'Не удалось установить скорость {goal.rate}: {exc}')
        if goal.language:
            try:
                self._speech_client.set_language(goal.language)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning(f'Не удалось установить язык {goal.language}: {exc}')
        if goal.punctuation_mode:
            self._set_punctuation(goal.punctuation_mode)

    def _set_punctuation(self, mode: str) -> None:
        if self._speech_client is None or speechd is None:
            return

        normalized = mode.strip().upper()
        mapping = {
            'NONE': getattr(speechd.PunctuationMode, 'NONE', None),
            'SOME': getattr(speechd.PunctuationMode, 'SOME', None),
            'ALL': getattr(speechd.PunctuationMode, 'ALL', None),
        }
        target = mapping.get(normalized)
        if target is None:
            self.get_logger().warning(
                f"Неизвестный режим пунктуации '{mode}', доступные значения: NONE, SOME, ALL"
            )
            return
        try:
            self._speech_client.set_punctuation(target)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f'Не удалось установить режим пунктуации {mode}: {exc}')

    def _wait_until_done(self, goal_handle, *, text: str) -> bool:
        """Дождаться завершения синтеза, если API speech-dispatcher это поддерживает."""
        if self._speech_client is None:
            return False

        wait_methods = [
            getattr(self._speech_client, 'wait_till_done', None),
            getattr(self._speech_client, 'wait_end', None),
        ]
        for wait_method in wait_methods:
            if wait_method is None:
                continue
            try:
                wait_method()
                return True
            except TypeError:
                # wait_end может требовать timeout, пробуем с простым ожиданием
                try:
                    wait_method(timeout=0.1)  # type: ignore[call-arg]
                except Exception:  # noqa: BLE001
                    continue
                return True
            except Exception as exc:  # noqa: BLE001
                self.get_logger().debug(f'wait_* завершился с ошибкой: {exc}')

        is_speaking = getattr(self._speech_client, 'is_speaking', None)
        start = time.monotonic()
        max_wait = max(2.0, min(len(text) / 6.0, 60.0))
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                return False
            if is_speaking is not None:
                try:
                    if not is_speaking():
                        return True
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().debug(f'is_speaking завершился с ошибкой: {exc}')
                    break
            if (time.monotonic() - start) > max_wait:
                return False
            time.sleep(0.05)
        return False

    def _stop_synthesis(self) -> None:
        if self._speech_client is None:
            return
        stop_methods = [
            getattr(self._speech_client, 'stop', None),
            getattr(self._speech_client, 'cancel', None),
        ]
        for method in stop_methods:
            if method is None:
                continue
            try:
                method()
                break
            except Exception as exc:  # noqa: BLE001
                self.get_logger().debug(f'Ошибка остановки синтеза: {exc}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TextToSpeechServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
