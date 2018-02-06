## Установка
Для запуска сцены необходимо скачать [V-REP PRO EDU](http://www.coppeliarobotics.com/downloads.html), распаковать и установить: 

```
$ cd /path/to/V-REP_PRO_EDU_V3_4_0_Linux.tar.gz
$ tar -xvf V-REP_PRO_EDU_V3_4_0_Linux.tar.gz
$ sudo mv V-REP_PRO_EDU_V3_4_0_Linux /opt/vrep
```
После установки V-REP нужно поставить [V-REP ROSInterface](https://github.com/fferri/v_repExtRosInterface). Для этого понадобится `catkin_tools`, устанавливаем его и следуем инструкции. 

Создаем рабочее пространство:
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws
$ catkin init & catkin build
```

Скачиваем пакеты `vrep_ros_interface` и `brics_actuator` для успешной работы сцены V-REP:
```
$ cd ~/catkin_ws/src
$ git clone --recursive https://github.com/fferri/v_repExtRosInterface.git vrep_ros_interface
$ git clone https://github.com/wnowak/brics_actuator.git
```

Перейдем в пакет `vrep_ros_interface` и добавим в `meta/messages.txt`, `CMakeLists.txt`, `package.xml` несколько строчек: 

1. `meta/messages.txt`
```
brics_actuator/CartesianPose
brics_actuator/CartesianTwist
brics_actuator/CartesianVector
brics_actuator/CartesianWrench
brics_actuator/JointAccelerations
brics_actuator/JointConstraint
brics_actuator/JointImpedances
brics_actuator/JointPositions
brics_actuator/JointTorques
brics_actuator/JointValue
brics_actuator/JointVelocities
brics_actuator/Poison 
```

2. `CMakeLists.txt` В коде нужно добавить `brics_actuator`. В итоге получим:
```
find_package(catkin REQUIRED COMPONENTS
    roscpp rosmsg image_transport tf cv_bridge brics_actuator
)
```

3. `package.xml` В коде нужно добавить строчку: 
```
<build_depend>brics_actuator</build_depend>
```

Теперь установим нужные переменные окружения и соберем пакет:
```
$ export VREP_ROOT=/opt/vrep
$ catkin build
```

И добавим символьную ссылку на библиотеку в V-REP:
```
$ cd /opt/vrep
$ ln -s ~/catkin_ws/devel/lib/libv_repExtRosInterface.so
```

## Запуск
Для запуска необходимо установить и настроить `V-REP PRO EDU`.

1. Запуск ядра ROS
```
$ roscore
```

2. Запуск `V-REP`.
```
$ vrep
```
Важно, чтобы после запуска `ROS Interface` был успешно запущен. Это можно увидеть при запуске `V-REP`, в консоле должно быть отображено:
```
Plugin 'RosInterface': loading...
Plugin 'RosInterface': load succeeded.
```

3. Запуск основного ROS окружения
```
$ roslaunch arm_manipulation load_kr6r900sixx.launch
```

4. Запуск управляющего узла.
```
$ rosrun local_task_planner camera_test
```
Управляющий узел предоставляет тестирование основных функций работы манипулятора. На выбор предоставляется 4 режима. Режим 1 и 2 можно запускать независимо. Третий режим можно запускать только если уже были выполнены первые два. Четвертый, если первые три.