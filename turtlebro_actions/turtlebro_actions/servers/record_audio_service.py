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

import traceback
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
        # USB Audio Device (card 0, device 0) выбран как дефолтный источник записи
        self.declare_parameter('device', 'plughw:0,0')
        self.declare_parameter('sample_rate', 48_000)
        self.declare_parameter('channels', 1)
        self.declare_parameter('period_size', 1024)
        self.declare_parameter('base_path', '/home/pi')
        self.declare_parameter('min_duration', 0.1)
        self.declare_parameter('capture_volume', 80)
        self.declare_parameter('capture_mixer', '')
        self.declare_parameter('digital_gain_db', 0.0)

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
            self.get_logger().error(
                f'Не удалось выполнить запись аудио: {exc}\n{traceback.format_exc()}'
            )
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
                f'Запрошена запись короче минимальной. Продолжительность увеличена до {min_duration:.2f} с'
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

        card_index = self._extract_card_index(device)
        self._apply_capture_gain(card_index)

        pcm = alsaaudio.PCM(
            type=alsaaudio.PCM_CAPTURE,
            mode=alsaaudio.PCM_NORMAL,
            device=device,
            channels=channels,
            rate=sample_rate,
            format=alsaaudio.PCM_FORMAT_S16_LE,
            periodsize=period_size,
        )

        bytes_per_frame = 2 * channels
        frames_required = int(round(duration * sample_rate))
        if frames_required <= 0:
            raise ValueError('Не удалось вычислить количество аудиофреймов для записи')

        captured_frames = 0
        buffer = bytearray()

        self.get_logger().debug(
            f'Начата запись: duration={duration:.2f}, samplerate={sample_rate}, '
            f'channels={channels}, device={device}, period={period_size}'
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

        buffer = self._apply_digital_gain(buffer)

        with wave.open(str(filepath), 'wb') as wav_file:
            wav_file.setnchannels(channels)
            wav_file.setsampwidth(2)
            wav_file.setframerate(sample_rate)
            wav_file.writeframes(buffer)

        peak, rms = self._compute_levels(buffer, channels)
        self.get_logger().info(f'Запись завершена: peak={peak:.3f}, rms={rms:.3f}, frames={frames_required}')
        if peak < 0.01:
            self.get_logger().warning(
                f'Уровень сигнала очень низкий (пик {peak:.3f}). Проверьте громкость/мьют микрофона или смените устройство.'
            )

        return filepath

    def _extract_card_index(self, device: str) -> Optional[int]:
        # Ожидаем строки вида hw:0,0 или plughw:1,0
        try:
            prefix, rest = device.split(':', 1)
            card_str = rest.split(',', 1)[0]
            return int(card_str)
        except Exception:
            return None

    def _apply_capture_gain(self, card_index: Optional[int]) -> None:
        mixer_name = self.get_parameter('capture_mixer').get_parameter_value().string_value
        target_volume = self.get_parameter('capture_volume').get_parameter_value().integer_value
        if target_volume <= 0:
            return
        if card_index is None:
            return
        try:
            # Выбираем первый подходящий микшер, если имя не задано
            mixers = alsaaudio.mixers(card_index)
            if not mixers:
                return
            control = mixer_name if mixer_name else None
            if control is None:
                for candidate in mixers:
                    low = candidate.lower()
                    if 'mic' in low or 'capture' in low:
                        control = candidate
                        break
                if control is None:
                    control = mixers[0]

            m = alsaaudio.Mixer(control=control, cardindex=card_index)
            m.setvolume(int(target_volume), alsaaudio.MIXER_CHANNEL_ALL)
            if hasattr(m, 'setrec'):
                try:
                    m.setrec(1, alsaaudio.MIXER_CHANNEL_ALL)
                except Exception:
                    pass
            self.get_logger().info(
                f'Установлена громкость {target_volume}% на микшере {control} (card {card_index})'
            )
        except Exception as exc:
            self.get_logger().warning(f'Не удалось настроить громкость capture: {exc}')

    def _apply_digital_gain(self, data: bytes) -> bytes:
        gain_db = self.get_parameter('digital_gain_db').get_parameter_value().double_value
        if abs(gain_db) < 1e-6:
            return data
        import array

        factor = 10 ** (gain_db / 20.0)
        samples = array.array('h')
        samples.frombytes(data)
        for i, sample in enumerate(samples):
            amplified = int(sample * factor)
            if amplified > 32767:
                amplified = 32767
            elif amplified < -32768:
                amplified = -32768
            samples[i] = amplified
        return samples.tobytes()

    def _compute_levels(self, data: bytes, channels: int) -> tuple[float, float]:
        if not data:
            return 0.0, 0.0
        import array

        samples = array.array('h')
        samples.frombytes(data)
        max_abs = max(abs(s) for s in samples) if samples else 0
        rms = (sum(s * s for s in samples) / len(samples)) ** 0.5 if samples else 0
        return max_abs / 32768.0, rms / 32768.0


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
