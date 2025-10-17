<img src="https://user-images.githubusercontent.com/57194638/201707251-5aa29404-2494-4e16-be4a-0cd821a1c0d9.png" width="800" height="300">

#  Метапакет turtlebro_extra для робота Turtlebro


## Установка пакета

```
cd ~/catkin_ws/src
git clone https://github.com/voltbro/turtlebro_extra
cd ..
catkin_make --pkg=turtlebro_extra
```

## Доступные пакеты для роботов компании "Братья Вольт" в пакете  turtlebro_extra

Список доступных для использования пакетов с небольшим описанием:

### turtlebro_actions

https://github.com/voltbro/turtlebro_extra/tree/master/turtlebro_actions

Представляет собой набор стандартных действий робота (проезд вперед, поворот и т.д.), оформленных в сервисы.

### turtlebro_py

https://github.com/voltbro/turtlebro_extra/tree/master/turtlebro_py

Питон-пакет для управления TurtleBro напрямую из Python без погружения в ROS. Предоставляет класс `TurtleBro` с командами движения, работы с сенсорами и периферией.

## Работа пакета *turtlebro_extra* на компьютере

В случае, если необходимо запустить работу одного из пакетов, входящих в метапакет **turtlebro_extra**, на компьютере, то для начала необходимо сконфигурировать собственное рабочее пространство:

```
cd
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```
Далее добавить ссылку на рабочее пространство в файл *bashrc*:
```
echo "source /home/$USER/catkin_ws/devel/setup.bash" >> ~/.bashrc source ~/.bashrc
```
