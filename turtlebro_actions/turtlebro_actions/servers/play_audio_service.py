#!/usr/bin/env python3

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
from __future__ import annotations

import threading
import wave
from pathlib import Path
from typing import Optional

import alsaaudio
import rclpy
from rclpy.node import Node

from turtlebro_interfaces.srv import PlayAudio


class PlayAudioService(Node):
    """Сервис воспроизведения WAV-файлов с использованием pyalsaaudio."""

    def __init__(self) -> None:
        super().__init__('play_audio_service')
        self.declare_parameter('device', 'default')
        self.declare_parameter('period_size', 2048)
        self.declare_parameter('base_path', '/home/pi')
        self.declare_parameter('allow_parallel', False)

        self._service = self.create_service(PlayAudio, 'play_audio', self.handle_play)
        self._playback_lock = threading.Lock()
        self._playback_thread: Optional[threading.Thread] = None

        self.get_logger().info('Сервис воспроизведения аудио инициализирован')

    def handle_play(self, request: PlayAudio.Request, response: PlayAudio.Response) -> PlayAudio.Response:
        try:
            filepath = self._normalise_filename(request.filename)
            self._validate_wav(filepath)
            if request.blocking:
                self._play_file(filepath, device_override=request.device)
                response.success = True
                response.message = f'Воспроизведение {filepath.name} завершено'
            else:
                self._start_async_playback(filepath, device_override=request.device)
                response.success = True
                response.message = f'Воспроизведение {filepath.name} запущено'
        except FileNotFoundError as exc:
            response.success = False
            response.message = str(exc)
            self.get_logger().warning(response.message)
        except alsaaudio.ALSAAudioError as exc:
            response.success = False
            response.message = f'Ошибка ALSA: {exc}'
            self.get_logger().error(response.message)
        except ValueError as exc:
            response.success = False
            response.message = str(exc)
            self.get_logger().warning(response.message)
        except RuntimeError as exc:
            response.success = False
            response.message = str(exc)
            self.get_logger().warning(response.message)
        except Exception as exc:  # noqa: BLE001 - диагностируем неожиданные ошибки
            response.success = False
            response.message = f'Не удалось воспроизвести файл: {exc}'
            self.get_logger().exception('Ошибка при воспроизведении аудио')
        return response

    def _normalise_filename(self, filename: str) -> Path:
        if not filename:
            raise ValueError('Имя файла для воспроизведения не задано')

        base_path = Path(self.get_parameter('base_path').get_parameter_value().string_value)
        base_path.mkdir(parents=True, exist_ok=True)

        target_path = Path(filename)
        if target_path.suffix.lower() != '.wav':
            target_path = target_path.with_suffix('.wav')

        if target_path.is_absolute():
            resolved = target_path.resolve()
        else:
            resolved = (base_path / target_path).resolve()

        base_resolved = base_path.resolve()
        if base_resolved not in resolved.parents and resolved != base_resolved:
            raise ValueError('Путь к файлу выходит за пределы разрешенной директории воспроизведения')

        if not resolved.exists():
            raise FileNotFoundError(f'Файл {resolved} не найден')

        return resolved

    def _validate_wav(self, filepath: Path) -> None:
        with wave.open(str(filepath), 'rb') as wav_file:
            channels = wav_file.getnchannels()
            sample_width = wav_file.getsampwidth()
            if sample_width != 2:
                raise ValueError('Поддерживаются только WAV-файлы с глубиной 16 бит (s16le)')
            if channels <= 0:
                raise ValueError('Некорректное количество каналов в WAV-файле')

    def _start_async_playback(self, filepath: Path, *, device_override: str) -> None:
        allow_parallel = self.get_parameter('allow_parallel').get_parameter_value().bool_value
        with self._playback_lock:
            if not allow_parallel and self._playback_thread and self._playback_thread.is_alive():
                raise RuntimeError('Проигрывание уже выполняется, дождитесь завершения или разрешите параллельный режим')

            thread = threading.Thread(
                target=self._playback_worker,
                args=(filepath, device_override),
                name=f'play_audio_{filepath.stem}',
                daemon=True,
            )
            self._playback_thread = thread
            thread.start()

    def _playback_worker(self, filepath: Path, device_override: str) -> None:
        try:
            self._play_file(filepath, device_override=device_override)
            self.get_logger().info(f'Воспроизведение {filepath.name} завершено')
        except Exception:  # noqa: BLE001 - логирование фатальных ошибок в фоновом потоке
            self.get_logger().exception('Ошибка во время фонового воспроизведения %s', filepath)
        finally:
            current_thread = threading.current_thread()
            with self._playback_lock:
                if self._playback_thread is current_thread:
                    self._playback_thread = None

    def _play_file(self, filepath: Path, *, device_override: str) -> None:
        device_param = self.get_parameter('device').get_parameter_value().string_value
        device = device_override or device_param
        if not device:
            raise ValueError('Не задано ALSA-устройство для воспроизведения (device)')

        period_size = self.get_parameter('period_size').get_parameter_value().integer_value
        if period_size <= 0:
            raise ValueError('Параметр period_size должен быть положительным')

        with wave.open(str(filepath), 'rb') as wav_file:
            channels = wav_file.getnchannels()
            rate = wav_file.getframerate()
            sample_width = wav_file.getsampwidth()
            if sample_width != 2:
                raise ValueError('Поддерживаются только WAV-файлы с глубиной 16 бит (s16le)')

            pcm = alsaaudio.PCM(type=alsaaudio.PCM_PLAYBACK, mode=alsaaudio.PCM_NORMAL, device=device)
            try:
                pcm.setchannels(channels)
                pcm.setrate(rate)
                pcm.setformat(alsaaudio.PCM_FORMAT_S16_LE)
                pcm.setperiodsize(period_size)

                data = wav_file.readframes(period_size)
                while data:
                    pcm.write(data)
                    data = wav_file.readframes(period_size)
            finally:
                pcm.close()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = PlayAudioService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Сервис воспроизведения аудио остановлен пользователем')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
