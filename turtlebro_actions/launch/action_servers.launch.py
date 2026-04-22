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
"""Запуск всех Action-/сервис-серверов пакета ``turtlebro_actions``.

Ниже каждый ``Node`` явно объявляет **все** свои ROS-параметры со значениями по
умолчанию. Это служит и конфигурацией, и документацией: чтобы переопределить
любой параметр, достаточно поменять значение в нужном словаре ``parameters``.

Разбиение по ноду:

* ``move_server`` / ``move_linear_server`` — оба реализуют action ``action_move``
  (прямолинейное перемещение). Активировать **ровно один** из двух.
  ``move_linear_server`` дополнительно стабилизирует курс по yaw из ``/odom``.
* ``rotate_server`` — action ``action_rotate``, поворот вокруг оси Z.
* ``photo_service`` / ``record_audio_service`` / ``play_audio_service`` /
  ``text_to_speech_server`` — сервисы периферии.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # -------------------------------------------------------------------------
    # Параметры движения (move_server / move_linear_server)
    # -------------------------------------------------------------------------
    # odom_topic      : топик одометрии, из которого берутся поза и orientation.
    # ramp_time_sec   : целевое время разгона и время торможения в секундах
    #                   для трапециевидного профиля. Если общий путь короче,
    #                   чем (v_min + v_max) * ramp_time_sec, робот не успевает
    #                   выйти на максимальную скорость — профиль становится
    #                   треугольным автоматически.
    move_common_params = {
        'odom_topic': '/odom',
        'ramp_time_sec': 2.0,
    }

    # Дополнительные параметры move_linear_server:
    # max_angular_z           : верхний клэмп |ω| на выходе PID-коррекции курса,
    #                           рад/с. Ограничивает «рывки» при больших ошибках.
    # linear_pid.kp/ki/kd     : коэффициенты PID-контура «ошибка yaw → ω».
    # linear_pid.integral_limit : насыщение интегральной составляющей (рад·с).
    #                             0.0 отключает anti-windup клэмп.
    move_linear_params = {
        **move_common_params,
        'max_angular_z': 0.5,
        'linear_pid.kp': 5.0,
        'linear_pid.ki': 1.5,
        'linear_pid.kd': 0.0,
        'linear_pid.integral_limit': 0.0,
    }

    # -------------------------------------------------------------------------
    # Параметры поворота (rotate_server)
    # -------------------------------------------------------------------------
    # odom_topic    : как в move_*.
    # ramp_time_sec : как в move_*, но применяется к профилю угловой скорости.
    rotate_params = {
        'odom_topic': '/odom',
        'ramp_time_sec': 2.0,
    }

    # -------------------------------------------------------------------------
    # Параметры записи звука (record_audio_service)
    # -------------------------------------------------------------------------
    # device          : ALSA-устройство захвата по умолчанию
    #                   (request.device имеет приоритет, если непусто).
    # sample_rate     : частота дискретизации в Гц.
    # channels        : число каналов (1 = моно, 2 = стерео).
    # period_size     : размер ALSA-периода в фреймах — баланс задержки/CPU.
    # base_path       : корневая папка для относительных путей файлов и
    #                   автосоздаётся при первой записи.
    # min_duration    : минимально допустимая длительность запроса в секундах;
    #                   более короткие цели отклоняются.
    # capture_volume  : целевой уровень захвата 0..100; применяется через
    #                   ``capture_mixer`` при старте записи.
    # capture_mixer   : имя ALSA-миксера для управления громкостью (пусто =
    #                   автоопределение по устройству).
    # digital_gain_db : дополнительный цифровой гейн после записи, дБ.
    record_audio_params = {
        'device': 'plughw:0,0',
        'sample_rate': 48_000,
        'channels': 1,
        'period_size': 1024,
        'base_path': '/home/pi',
        'min_duration': 0.1,
        'capture_volume': 80,
        'capture_mixer': '',
        'digital_gain_db': 0.0,
    }

    # -------------------------------------------------------------------------
    # Параметры воспроизведения звука (play_audio_service)
    # -------------------------------------------------------------------------
    # device         : ALSA-устройство воспроизведения по умолчанию.
    # period_size    : размер ALSA-периода в фреймах при проигрывании.
    # base_path      : корневая папка для относительных путей WAV-файлов.
    # allow_parallel : разрешить одновременное воспроизведение нескольких
    #                  файлов (по умолчанию выключено — второй запрос отклоняется,
    #                  пока первый играет).
    play_audio_params = {
        'device': 'default',
        'period_size': 2048,
        'base_path': '/home/pi',
        'allow_parallel': False,
    }

    return LaunchDescription(
        [
            # Для прямолинейного перемещения доступны два Action-сервера на
            # одном действии `action_move`: активировать ровно один.
            #   * move_server          — базовый, без коррекции курса.
            #   * move_linear_server   — со стабилизацией yaw по /odom через PID.
            # По умолчанию включён move_linear_server, move_server закомментирован.
            # Node(
            #     package='turtlebro_actions',
            #     executable='move_server.py',
            #     name='move_server',
            #     output='log',
            #     respawn=True,
            #     parameters=[move_common_params],
            # ),
            Node(
                package='turtlebro_actions',
                executable='move_linear_server.py',
                name='move_linear_server',
                output='log',
                respawn=True,
                parameters=[move_linear_params],
            ),
            Node(
                package='turtlebro_actions',
                executable='rotate_server.py',
                name='rotate_server',
                output='log',
                respawn=True,
                parameters=[rotate_params],
            ),
            Node(
                package='turtlebro_actions',
                executable='photo_service.py',
                name='photo_service',
                output='log',
                respawn=True,
            ),
            Node(
                package='turtlebro_actions',
                executable='record_audio_service.py',
                name='record_audio_service',
                output='log',
                respawn=True,
                parameters=[record_audio_params],
            ),
            Node(
                package='turtlebro_actions',
                executable='play_audio_service.py',
                name='play_audio_service',
                output='log',
                respawn=True,
                parameters=[play_audio_params],
            ),
            Node(
                package='turtlebro_actions',
                executable='text_to_speech_server.py',
                name='text_to_speech_server',
                output='log',
                respawn=True,
            ),
        ]
    )
