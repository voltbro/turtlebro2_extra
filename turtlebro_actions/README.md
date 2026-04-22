# Установка пакета
Пакет входит в сборку метапакета turtlebro_extra и устанавливается автоматически при установке этого пакета. Если требуется поставить `turtlebro_actions` отдельно, выполните на роботе:

```
mkdir -p ~/turtlebro_ws/src
cd ~/turtlebro_ws/src
git clone https://github.com/voltbro/turtlebro_extra
cd ..
colcon build --packages-select turtlebro_actions
source install/setup.bash
```

# turtlebro_actions

## Запуск action-серверов

```
ros2 launch turtlebro_actions action_servers.launch.py
```

Лаунч поднимает серверы движения (`MoveServer` либо `MoveLinearServer` — см. ниже) и поворота (`RotateServer`), сервис фотографирования (`PhotoService`), сервисы записи и воспроизведения звука (`RecordAudioService`, `PlayAudioService`), а также Action-сервер синтеза речи (`TextToSpeechServer`) c использованием RHVoice через speech-dispatcher.

Все ROS-параметры, которые можно переопределить для каждого узла, объявлены в [`launch/action_servers.launch.py`](launch/action_servers.launch.py) с комментариями. Менять значения можно прямо там, либо через CLI:

```
ros2 run turtlebro_actions rotate_server.py --ros-args -p ramp_time_sec:=1.0
ros2 param set /move_linear_server linear_pid.kp 3.0
```

## Прямолинейное движение (`action_move`)

На одном действии `action_move` доступны **два** сервера — активировать нужно **ровно один** (в лаунче оставить соответствующий `Node`):

- `MoveServer` ([`servers/move_server.py`](turtlebro_actions/servers/move_server.py)) — базовый, трапециевидный профиль линейной скорости, без коррекции курса.
- `MoveLinearServer` ([`servers/move_linear_server.py`](turtlebro_actions/servers/move_linear_server.py)) — тот же профиль плюс PID-контур стабилизации yaw по orientation из одометрии. Удерживает курс, зафиксированный в момент старта цели.

По умолчанию в лаунче включён `MoveLinearServer`.

## Поворот (`action_rotate`, `RotateServer`)

- **До 180° включительно** (по модулю цели): прогресс считается **по углу относительно yaw в начале цели** (одометрия `nav_msgs/Odometry` → yaw из ориентации). Такой режим **точнее** для типичных поворотов: меньше накопление ошибки по тикам управления.
- **Свыше 180°**: используется **суммирование малых приращений** yaw между обновлениями одометрии (развёртка угла в `[-π, π]`). Для нескольких полных оборотов это необходимо, но шум и дискретность одометрии **могут накапливаться**, поэтому итоговый угол **менее точен**, чем для целей ≤ 180°.

Практический вывод: для требовательной точности лучше задавать повороты **порциями до 180°**; большие углы разбивать на несколько целей или закладывать больший допуск.

## ROS-параметры серверов движения

Значения по умолчанию и подробные комментарии см. в [`launch/action_servers.launch.py`](launch/action_servers.launch.py). Таблица ниже — сводка.

### `MoveServer` и `MoveLinearServer`

| Параметр | Тип | По умолчанию | Смысл |
|---|---|---|---|
| `odom_topic` | string | `/odom` | Топик одометрии для отслеживания позиции и yaw. |
| `ramp_time_sec` | double | `2.0` | Целевое время разгона и время торможения в секундах. См. раздел «Трапециевидный профиль скорости» ниже. |

Дополнительно у `MoveLinearServer`:

| Параметр | Тип | По умолчанию | Смысл |
|---|---|---|---|
| `max_angular_z` | double | `0.5` | Верхний клэмп `|ω_z|` на выходе PID, рад/с. |
| `linear_pid.kp` | double | `5.0` | Пропорциональная составляющая контура «ошибка yaw → ω_z». |
| `linear_pid.ki` | double | `1.5` | Интегральная составляющая. |
| `linear_pid.kd` | double | `0.0` | Дифференциальная составляющая. |
| `linear_pid.integral_limit` | double | `0.0` | Насыщение интегрального накопителя (рад·с). `0.0` отключает anti-windup клэмп. |

`ramp_time_sec`, `max_angular_z` и коэффициенты PID считываются **один раз на старте цели** (`execute_callback`) и не меняются до её завершения. Поэтому динамические правки через `ros2 param set` применяются со следующей цели, а не на лету.

### `RotateServer`

| Параметр | Тип | По умолчанию | Смысл |
|---|---|---|---|
| `odom_topic` | string | `/odom` | Топик одометрии для отслеживания yaw. |
| `ramp_time_sec` | double | `2.0` | Целевое время разгона и торможения угловой скорости. |

## Трапециевидный профиль скорости

Профиль реализован в `compute_trapezoidal_speed` из [`turtlebro_actions.utils.motion_profile`](turtlebro_actions/utils/motion_profile.py). Скорость растёт линейно по пройденному пути от `min_speed` до `max_speed`, удерживается на `max_speed`, затем симметрично спадает до `min_speed`.

Длина участка разгона вычисляется из заданного времени `ramp_time_sec` (параметр ноды) по формуле:

```
ramp_distance = 0.5 * (min_speed + max_speed) * ramp_time_sec
```

и автоматически ограничивается значением `total_distance / 2`: если общий путь короче суммарного разгона + торможения, профиль становится **треугольным** и робот не выходит на `max_speed`.

`max_speed` берётся из поля `speed` цели Action; если `speed == 0` (или опущено) — используется дефолт, прошитый в исходник соответствующего сервера.

### Константы профиля (в исходниках серверов)

Меняются правкой кода и пересборкой пакета. Используются для расчёта `max_speed` / `min_speed` в каждой цели.

#### [`servers/rotate_server.py`](turtlebro_actions/servers/rotate_server.py)

| Константа | Значение | Смысл |
|---|---|---|
| `_ROTATE_DEFAULT_MAX_SPEED` | `0.9` | Максимальная угловая скорость (рад/с), если в цели `speed == 0`. |
| `_ROTATE_MIN_SPEED_FRAC_OF_MAX` | `0.1` | Минимальная скорость на фазах разгона/торможения: не меньше этой доли от выбранного `max_speed`. |
| `_ROTATE_MIN_SPEED_FLOOR` | `0.03` | Нижний предел минимальной скорости (рад/с), чтобы на малых `max_speed` не падать ниже разумного порога. |
| `_ROTATE_DEFAULT_RAMP_TIME_SEC` | `2.0` | Дефолт ROS-параметра `ramp_time_sec`. |

#### [`servers/move_server.py`](turtlebro_actions/servers/move_server.py) и [`servers/move_linear_server.py`](turtlebro_actions/servers/move_linear_server.py)

| Константа | Значение | Смысл |
|---|---|---|
| `_MOVE_DEFAULT_MAX_SPEED` | `0.09` | Линейная скорость (м/с), если в цели `speed == 0`. |
| `_MOVE_MIN_SPEED_FRAC_OF_MAX` | `0.2` | Доля от `max_speed` для минимальной скорости на трапеции. |
| `_MOVE_MIN_SPEED_FLOOR` | `0.02` | Нижний предел минимальной скорости (м/с). |
| `_MOVE_DEFAULT_RAMP_TIME_SEC` | `2.0` | Дефолт ROS-параметра `ramp_time_sec`. |

Итоговая минимальная скорость вдоль траектории (одинаково для move и rotate, с соответствующими константами):

```
min_speed = min(max(max_speed * MIN_SPEED_FRAC_OF_MAX, MIN_SPEED_FLOOR), max_speed)
```

## ROS-параметры сервисов аудио

Значения по умолчанию и комментарии — в том же `launch/action_servers.launch.py`. Сводка:

### `RecordAudioService`

| Параметр | Тип | По умолчанию | Смысл |
|---|---|---|---|
| `device` | string | `plughw:0,0` | ALSA-устройство захвата. `request.device` в запросе имеет приоритет, если непустой. |
| `sample_rate` | int | `48000` | Частота дискретизации, Гц. |
| `channels` | int | `1` | Число каналов (1 — моно). |
| `period_size` | int | `1024` | Размер ALSA-периода во фреймах. Баланс задержка/CPU. |
| `base_path` | string | `/home/pi` | Корневая папка для относительных путей WAV-файлов. |
| `min_duration` | double | `0.1` | Минимально допустимая длительность запроса, сек. |
| `capture_volume` | int | `80` | Целевой уровень захвата 0..100. Применяется через `capture_mixer`. |
| `capture_mixer` | string | `""` | Имя ALSA-миксера. Пусто — автоопределение по устройству. |
| `digital_gain_db` | double | `0.0` | Дополнительный цифровой гейн после записи, дБ. |

### `PlayAudioService`

| Параметр | Тип | По умолчанию | Смысл |
|---|---|---|---|
| `device` | string | `default` | ALSA-устройство воспроизведения. |
| `period_size` | int | `2048` | Размер ALSA-периода во фреймах. |
| `base_path` | string | `/home/pi` | Корневая папка для относительных путей WAV-файлов. |
| `allow_parallel` | bool | `false` | Разрешить одновременное воспроизведение нескольких файлов. По умолчанию второй запрос отклоняется, пока играет первый. |

У `PhotoService` и `TextToSpeechServer` ROS-параметров нет.

## Синтез речи (RHVoice)

- Зависимости: `python3-speechd` и `speech-dispatcher`. На роботе также должна быть установлена и настроена RHVoice (список голосов, язык по умолчанию и т.д.).
- Action-сервер предоставляет действие `text_to_speech`. Цель включает текст, имя голоса, язык, скорость и режим произношения пунктуации (NONE/SOME/ALL).
- Пример клиента: `ros2 run turtlebro_actions text_to_speech_client.py`. Можно передать собственный текст, отредактировав скрипт или создав свою ноду.
- Для проверки доступных голосов выполните:

  ```python
  python3 - <<'PY'
  import speechd
  client = speechd.SSIPClient('voices_probe')
  client.set_output_module('rhvoice')
  print(client.list_synthesis_voices())
  client.close()
  PY
  ```

## Структура Python-модулей

- Серверные узлы доступны в `turtlebro_actions.servers`.
- Готовые учебные клиенты вынесены в `turtlebro_actions.examples` и могут служить образцами для собственного кода.
- Общие утилиты находятся в `turtlebro_actions.utils`.

Импортируйте нужные классы напрямую:

```python
from turtlebro_actions import MoveClient, RotateServer
```

## Фото и звук

- `PhotoService` предоставляет сервис `get_photo` и возвращает последнее сжатое изображение с фронтальной камеры.
- `RecordAudioService` обслуживает сервис `record_audio` и записывает звук с ALSA-устройства (по умолчанию `plughw:0,0` — USB Audio Device) через библиотеку `pyalsaaudio` в WAV-файл под `/home/pi`. Для работы требуется пакет `python3-alsaaudio`.
- `PlayAudioService` обслуживает сервис `play_audio` и воспроизводит WAV-файлы посредством `pyalsaaudio`. По умолчанию файлы ищутся в `/home/pi`, каталог можно поменять параметром `base_path`.

Параметры сервиса аудиозаписи:

- `duration` — длительность записи в секундах (> 0);
- `filename` — базовое имя файла (расширение задаётся отдельно);
- `device` — строковое имя ALSA-устройства (например, `plughw:0,0`), необязательный параметр, по умолчанию берётся из параметра `device` узла.

Чтобы подобрать идентификатор устройства записи, воспользуйтесь одной из утилит:

- `arecord -l` — покажет физические устройства и их номера (`card X`, `device Y`), после чего можно сформировать строку `plughw:X,Y` или `hw:X,Y`;
- Python: `python3 - <<'PY'
import alsaaudio
print(alsaaudio.cards())
print(alsaaudio.pcms(alsaaudio.PCM_CAPTURE))
PY`

Аналогично для устройства воспроизведения:

- `aplay -l` — отобразит доступные ALSA-устройства вывода и их номера (`card X`, `device Y`), которые затем можно использовать как `hw:X,Y`;
- Python: `python3 - <<'PY'
import alsaaudio
print(alsaaudio.pcms(alsaaudio.PCM_PLAYBACK))
PY`

Изменить устройство по умолчанию можно либо при вызове сервиса (`device: 'plughw:2,0'`), либо через параметр узла: `ros2 param set /record_audio_service device plughw:2,0`.

Например, чтобы записать 5 секунд в WAV:

```
ros2 service call /record_audio turtlebro_interfaces/srv/RecordAudio "{duration: 5.0, filename: 'lesson', device: ''}"
```

### Воспроизведение WAV

По умолчанию `PlayAudioService` ищет файлы в `/home/pi` и автоматически дополняет имя расширением `.wav`, если оно не указано. Устройство воспроизведения можно задать параметром `device` или напрямую в запросе.

Запустить проигрывание в фоне:

```
ros2 service call /play_audio turtlebro_interfaces/srv/PlayAudio "{filename: 'alarm', blocking: false, device: ''}"
```

Чтобы дождаться завершения воспроизведения, установите `blocking: true` — сервис вернёт ответ после окончания проигрывания.
