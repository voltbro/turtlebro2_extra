# turtlebro_patrol (ROS 2)

Пакет патрулирования для TurtleBro под ROS 2.

## Что делает

- Принимает команды через сервис `/patrol_control`.
- Отправляет цели в `navigate_to_pose` (Nav2).
- Публикует достигнутую точку в `/patrol_control/reached`.
- Может вызывать callback-сервис в каждой точке (если задан `point_callback_service`).

## Команды управления

Сервис: `/patrol_control`  
Тип: `turtlebro_patrol/srv/PatrolControlCallback`

Допустимые команды:

1. `start` — начать патрульный цикл с первой точки.
2. `pause` — остановить патрулирование (без новой цели).
3. `resume` — продолжить с текущей точки.
4. `home` — отправить робота в домашнюю точку.
5. `shutdown` — остановить патруль и завершить узел.

## Конфигурация точек

Файл с точками:

`share/turtlebro_patrol/data/goals.toml`

Пример:

```toml
[home]
pose = {x = 0, y = 0, theta = 0}

[[patrolling]]
name = "Goal1"
pose = {x = 0.5, y = 0, theta = 45}
```

## Запуск

Только узел патрулирования:

```bash
ros2 launch turtlebro_patrol patrol_run.launch.py
```

С опциональным запуском навигации:

```bash
ros2 launch turtlebro_patrol patrol.launch.py run_navigation:=true
```

С callback-сервисом точки:

```bash
ros2 launch turtlebro_patrol patrol_run.launch.py point_callback_service:=my_service_name
```
