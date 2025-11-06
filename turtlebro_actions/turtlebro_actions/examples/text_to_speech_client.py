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
"""Пример Action-клиента, отправляющего текст на синтез речи."""

from __future__ import annotations

import sys
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node

from turtlebro_interfaces.action import TextToSpeech


class TextToSpeechClient:
    """Простой клиент синтеза речи с параметрами по умолчанию для RHVoice."""

    def __init__(self, node: Node, wait_timeout: float = 5.0) -> None:
        self._node = node
        self._client = ActionClient(node, TextToSpeech, 'text_to_speech')
        if not self._client.wait_for_server(timeout_sec=wait_timeout):
            raise RuntimeError('Action-сервер text_to_speech недоступен')
        self._node.get_logger().info('Клиент text_to_speech подключен')

    def speak(
        self,
        text: str,
        *,
        voice: str = 'Vsevolod',
        rate: int = 20,
        language: str = 'ru',
        punctuation_mode: str = 'SOME',
    ) -> TextToSpeech.Result:
        goal = TextToSpeech.Goal()
        goal.text = text
        goal.voice = voice
        goal.rate = int(rate)
        goal.language = language
        goal.punctuation_mode = punctuation_mode

        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self._node, send_future)

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError('Цель синтеза речи отклонена сервером')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self._node, result_future)

        goal_result = result_future.result()
        if goal_result is None or goal_result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError('Синтез речи завершился неуспешно')

        return goal_result.result


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = Node('text_to_speech_example_client')
    try:
        client = TextToSpeechClient(node)
        result = client.speak('Проверка синтезатора речи')
        node.get_logger().info(f'Ответ сервера: success={result.success}, message={result.message}')
    except Exception as exc:  # noqa: BLE001
        node.get_logger().error(f'Сбой клиента синтеза речи: {exc}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
