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

import wave
from pathlib import Path
from typing import Optional

import alsaaudio
import rclpy
from rclpy.node import Node

from turtlebro_interfaces.srv import RecordAudio


class RecordAudioService(Node):
    """Сервис записи аудио на базе pyalsaaudio."""

    def __init__(self) -> None:
        super().__init__('record_audio_service')
        self.declare_parameter('device', 'hw:1,0')
        self.declare_parameter('sample_rate', 48_000)
        self.declare_parameter('channels', 1)
        self.declare_parameter('period_size', 1024)
        self.declare_parameter('base_path', '/home/pi')
        self.declare_parameter('min_duration', 0.1)

        self._service = self.create_service(RecordAudio, 'record_audio', self.handle_record)
        self.get_logger().info('Сервис записи аудио инициализирован')

    def handle_record(self, request: RecordAudio.Request, response: RecordAudio.Response) -> RecordAudio.Response:
        try:
            file_path = self._record_audio(
                duration=request.duration,
                filename=request.filename,
                device_override=request.device,
            )
        except alsaaudio.ALSAAudioError as exc:
            response.success = False
            response.message = f'Ошибка ALSA: {exc}'
            self.get_logger().error(response.message)
        except ValueError as exc:
            response.success = False
            response.message = str(exc)
            self.get_logger().warning(response.message)
        except Exception as exc:  # noqa: BLE001 - логируем неожиданные сбои
            response.success = False
            response.message = f'Не удалось записать звук: {exc}'
            self.get_logger().exception('Не удалось выполнить запись аудио')
        else:
            response.success = True
            response.filepath = str(file_path)
            response.message = 'Запись завершена успешно'
            self.get_logger().info(f'Аудиофайл сохранен: {file_path}')
        return response

    def _normalise_filename(self, filename: str) -> Path:
        base_path = Path(self.get_parameter('base_path').get_parameter_value().string_value)
        base_path.mkdir(parents=True, exist_ok=True)

        if not filename:
            filename = 'turtlebro_sound'

        target_path = Path(filename)
        if target_path.suffix.lower() != '.wav':
            target_path = target_path.with_suffix('.wav')
        if target_path.is_absolute():
            resolved = target_path.resolve()
        else:
            resolved = (base_path / target_path).resolve()

        base_resolved = base_path.resolve()
        if base_resolved not in resolved.parents and resolved != base_resolved:
            raise ValueError('Путь к файлу выходит за пределы разрешенной директории записи')
        return resolved

    def _record_audio(
        self,
        *,
        duration: float,
        filename: str,
        device_override: str,
    ) -> Path:
        min_duration = self.get_parameter('min_duration').get_parameter_value().double_value
        if duration <= 0.0:
            raise ValueError('Продолжительность записи должна быть больше 0 секунд')
        if duration < min_duration:
            self.get_logger().warning(
                'Запрошена запись короче минимальной. Продолжительность будет увеличена до %.2f с',
                min_duration,
            )
            duration = float(min_duration)

        sample_rate = self.get_parameter('sample_rate').get_parameter_value().integer_value
        channels = self.get_parameter('channels').get_parameter_value().integer_value
        period_size = self.get_parameter('period_size').get_parameter_value().integer_value
        if sample_rate <= 0 or channels <= 0:
            raise ValueError('Параметры sample_rate и channels должны быть положительными')
        if period_size <= 0:
            raise ValueError('Параметр period_size должен быть положительным')

        filepath = self._normalise_filename(filename)

        device_param = self.get_parameter('device').get_parameter_value().string_value
        device = device_override or device_param
        if not device:
            raise ValueError('Не задано ALSA-устройство для записи (device)')

        pcm = alsaaudio.PCM(type=alsaaudio.PCM_CAPTURE, mode=alsaaudio.PCM_NORMAL, device=device)
        pcm.setchannels(channels)
        pcm.setrate(sample_rate)
        pcm.setformat(alsaaudio.PCM_FORMAT_S16_LE)
        pcm.setperiodsize(period_size)

        bytes_per_frame = 2 * channels
        frames_required = int(round(duration * sample_rate))
        if frames_required <= 0:
            raise ValueError('Не удалось вычислить количество аудиофреймов для записи')

        captured_frames = 0
        buffer = bytearray()

        self.get_logger().debug(
            'Начата запись: duration=%.2f, samplerate=%d, channels=%d, device=%s, period=%d',
            duration,
            sample_rate,
            channels,
            device,
            period_size,
        )

        try:
            while captured_frames < frames_required:
                length, data = pcm.read()
                if length <= 0:
                    continue
                buffer.extend(data)
                captured_frames += length
        finally:
            pcm.close()

        extra_frames = captured_frames - frames_required
        if extra_frames > 0:
            extra_bytes = extra_frames * bytes_per_frame
            if extra_bytes < len(buffer):
                del buffer[-extra_bytes:]

        with wave.open(str(filepath), 'wb') as wav_file:
            wav_file.setnchannels(channels)
            wav_file.setsampwidth(2)
            wav_file.setframerate(sample_rate)
            wav_file.writeframes(buffer)

        return filepath


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = RecordAudioService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Сервис записи аудио остановлен пользователем')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
