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

Лаунч поднимает серверы движения (`MoveServer`) и поворота (`RotateServer`), а также сервис фотографирования (`PhotoService`).

## Структура Python-модулей

- Серверные узлы доступны в `turtlebro_actions.servers`.
- Готовые учебные клиенты вынесены в `turtlebro_actions.examples` и могут служить образцами для собственного кода.
- Общие утилиты находятся в `turtlebro_actions.utils`.

Импортируйте нужные классы напрямую:

```python
from turtlebro_actions import MoveClient, RotateServer
```

## Радиомост

Командный режим радиоуправления и файл `radio.launch.py` вынесены в отдельный пакет `turtlebro_radio`. Актуальную инструкцию по радиомосту смотрите в `turtlebro_radio/README.md`.
