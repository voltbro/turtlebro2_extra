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

Лаунч поднимает серверы движения (`MoveServer`) и поворота (`RotateServer`), сервис фотографирования (`PhotoService`), сервисы записи и воспроизведения звука (`RecordAudioService`, `PlayAudioService`), а также Action-сервер синтеза речи (`TextToSpeechServer`) c использованием RHVoice через speech-dispatcher.

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
