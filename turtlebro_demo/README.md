# turtlebro_demo

`turtlebro_demo` — пакет-представление, показывающий ключевые возможности `turtlebro_py`. Узел произносит, что собирается сделать, и затем выполняет действие: меняет подсветку, проверяет дальномер, едет вперёд/назад, разворачивается, делает снимок, записывает и воспроизводит звук.

## Подготовка

1. Соберите и переустановите пакеты `turtlebro_py`, `turtlebro_actions` и новый пакет демонстратора:

   ```bash
   colcon build --packages-select turtlebro_py turtlebro_actions turtlebro_demo --symlink-install
   source install/setup.bash
   ```

2. Запустите сервисы из `turtlebro_actions` и микроконтроллер:

   ```bash
   ros2 launch turtlebro_actions action_servers.launch.py
   ros2 launch turtlebro_py micro_ros_agent.launch.py dev:=/dev/ttyACM0 baud:=115200
   ```

3. Убедитесь, что робот стоит на ровной поверхности с свободным пространством спереди не менее чем на расстояние `forward_distance_m`.

## Запуск

Проще всего воспользоваться готовым лаунчем:

```bash
ros2 launch turtlebro_demo demo.launch.py \
  forward_distance_m:=0.6 rotation_angle_deg:=90 \
  photo_filename:=demo_photo record_duration_s:=4.0
```

Узел `turtlebro_demo` проговаривает каждое действие (настройки голоса и темпа берутся из параметров `voice` и `speech_rate`). Доступные шаги:

1. приветствие и установка начальной подсветки;
2. демонстрация управления LED/подсветкой;
3. чтение расстояния с лидара;
4. движение вперёд и возврат назад;
5. разворот на заданный угол;
6. создание снимка и сохранение в `/home/pi/<photo_filename>.jpg`;
7. запись аудио и воспроизведение (если сервисы доступны);
8. финальное сообщение и выключение подсветки.

## Основные параметры

| Параметр | Назначение | Значение по умолчанию |
| --- | --- | --- |
| `forward_distance_m` / `backward_distance_m` | дистанция движения вперёд/назад (м) | `0.5` / `0.3` |
| `rotation_angle_deg` | угол разворота (градусы) | `90` |
| `distance_angle_deg` | направление измерения лидара (градусы 0…359) | `0` |
| `voice`, `speech_rate`, `speech_language` | голос/скорость/язык синтезатора RHVoice | `Vsevolod`, `15`, `ru` |
| `enable_led_show`, `enable_distance_report`, `enable_photo`, `enable_audio` | включение отдельных шагов | `true` |
| `photo_filename`, `record_filename` | имена файлов для фото и аудио (без расширения) | `demo_photo`, `demo_recording` |
| `record_duration_s`, `audio_device` | длительность записи и ALSA-устройство | `3.0`, пустая строка (использовать значение сервиса) |

Любой параметр можно переопределить через `ros2 run ... --ros-args -p <name>:=<value>` или через launch-аргументы (см. `launch/demo.launch.py`).

## Тесты

Для smoke-проверки импорта выполните:

```bash
colcon test --packages-select turtlebro_demo --event-handlers console_direct+
```

Тесты не взаимодействуют с оборудованием и лишь проверяют, что все модули импортируются.
