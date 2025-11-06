<img src="https://user-images.githubusercontent.com/57194638/201707251-5aa29404-2494-4e16-be4a-0cd821a1c0d9.png" width="800" height="300">

# Метапакет turtlebro_extra для робота TurtleBro (ROS 2)

## Установка и сборка

```
mkdir -p ~/turtlebro_ws/src
cd ~/turtlebro_ws/src
git clone https://github.com/voltbro/turtlebro_extra
cd ..
colcon build --packages-select turtlebro_actions turtlebro_py turtlebro_radio
source install/setup.bash
```

## Доступные пакеты

### turtlebro_actions

- `ros2 launch turtlebro_actions action_servers.launch.py` — поднимает action-серверы движения, поворота, фото-, аудио- и TTS-сервисы (RHVoice через speech-dispatcher).
- Готовые Python-клиенты доступны в модулях `turtlebro_actions.examples.*`, серверы — в `turtlebro_actions.servers.*`.

### turtlebro_radio

- `ros2 launch turtlebro_radio radio.launch.py port:=/dev/ttyUSB0` — стартует радиомост и позволяет передать параметры порта/скоростей.

### turtlebro_py

Python-библиотека с классами `TurtleBro` и `TurtleBroNav`, обеспечивающими команды движения, работу с сенсорами, камерой и озвучкой.

```python
from turtlebro_py import TurtleBro

tb = TurtleBro()
tb.forward(0.2)
tb.right(90)
tb.save_photo("demo")
tb.shutdown()
```

## Настройка среды

Добавьте workspace в `~/.bashrc`, чтобы ROS 2 окружение поднималось автоматически:

```
echo "source ~/turtlebro_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

После этого пакеты доступны через `ros2 run` и Python API. Дополнительные детали смотрите в README соответствующих пакетов.
