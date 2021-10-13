## Software installation on KRC4
EKI interface is required on both, KRC4 and NUC. Detailed information on how to set it up on KRC4 is presented [here](https://github.com/AlexeiOvcharov/kuka_experimental/tree/a915bf4e932990379c84164713e7ae11a24a2a13/kuka_eki_hw_interface/krl). Launch it on robot's controller.

## Software installation on NUC
ROS
```
sudo apt-get install ros-melodic-desktop-full
sudo apt-get install ros-melodic-joint-trajectory-controller
```
[librealsense](https://github.com/IntelRealSense/librealsense/blob/legacy/doc/installation.md)
[opencv](https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html)

Create a catkin workspace:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin build
```
Download ROS packages. All the scripts are stored [here](https://github.com/airalab/robot_painter/tree/test_branch). Clone the repository:
```
cd src
git clone --branch test_branch https://github.com/airalab/robot_painter
cd robot_painter
rm -rf scenes
mv * ../
cd ..
rmdir robot_painter
```
You may need some header files and libraries to make it all work correctly. Download them:
```
cd ~
git clone https://github.com/PaTara43/kuka_moveit_webots
cd kuka_moveit_webots
sudo mv headers/* usr/include/c++/7/
sudo mv libs/* /usr/local/lib/
cd ~
svn checkout https://github.com/PX4/Matrix/trunk/matrix
mv matrix -r /usr/include/c++/7/
sudo apt-get install ros-melodic-brics-actuator
cd ~/catkin_ws
catkin build
```
Add source command to `.bashrc` file:
```
echo “source ~/catkin_ws/devel/setup.bash” >> ~/.bashrc
source ~/.bashrc
```
Up to now you should be able to launch the scripts. If something goes wrong, try some [troubleshooting](https://github.com/airalab/robot_painter/issues).

## Filling in constants
First of all, the robot needs to know canvas location and orientation as well as the paint tin position. All of this is specified in `fake_painter_enviroment_tf/src/tf_broadcaster.cpp`. Let's take a look into it.
```
// Plane constants
const double A = -0.0641;
const double B = 0.0214;
const double C = 0.9977;
const double D = -0.2198;

// Canvas transform
const double px = 0.52;
const double py = -0.24;
const double qx = -0.011;
const double qy = -0.032;
const double qz = 0.0;
const double qw = 0.999;
```
These are the plane equation constants which specify canvas position in 3-D space. They are to be obtained during a calibration process described below. Next goes the paint.
```
colorTransform.transform.translation.x = 0.5;
colorTransform.transform.translation.y = 0.2;
colorTransform.transform.translation.z = 0.258;
```
These are paint tin coordinates. They also may be specified while calibrating. Canvas size is specified in
```
canvas.width = 0.5;
canvas.height = 0.4;
```
Several more important constants are stored in `local_task_planner/src/Drawing.cpp`:
```
const double COLOR_BOTLE_HEIGHT = 0.06;
const double COLOR_HEIGHT = 0.045;
const double HEIGHT_OFFSET = COLOR_BOTLE_HEIGHT - COLOR_HEIGHT + 0.02;
const double BRUSH_HEIGHT = 0.01;
const double BRUSH_WIDTH = 0.01;
```
Their names say it all, so fill them in according to the situation.

## Calibrating Gaka-Chu
The calibration process itself is pretty simple.

1) Start EKI interface on the KRC4:

Log in in 'AUT' mode, turn on drivers and launch the script `eki_hw_interface`.

2) Start EKI interface on the NUC
```
roslaunch kuka_eki_hw_interface test_hardware_interface.launch
```
It should output endless logs.

3) Start RViz
```
roslaunch kuka_moveit_config demo.launch
```

Try moving the end effector and clicking 'Plan and Execute'. The robot should move. On SmartPad go to **Display -> Actual position** and observe end effector's coordinate. Place a canvas horizontally to the robot base. Plug a brush into the brush holder and carefully move it till it barely touches the canvas. At this position, save end effector's coordinates. Repeat 12-15 times. Also, save the coordinates of the canvas center and paint tin.
When you have a set of coordinates, use [these](https://github.com/nakata5321/Matlab_scripts_gaka-chu) Matlab scripts to resolve the missing constants and quaternion. Paste them. Rebuild your workspace with
```
cd ~/catkin_workspace
rm -rf build logs devel
catkin build
```

## Testing Gaka-Chu calibration
When calibrated, Gaka-Chu needs to be tested by drawing the borders of canvas. To make him do so execute each in new terminal:
```
roslaunch kuka_eki_hw_interface test_hardware_interface.launch
roslaunch kuka_moveit_config demo.launch
rosrun fake_painter_enviroment_tf tf_broadcaster
rosrun local_task_planner draw_workspace
```

In terminal press "S" to perform testing. Robot's end effector should move right above the borders of the canvas and the brush should gently touch the canvas during the entire movement. If not so, try recalibrating. If the canvas model is rotated wrong, you can rotate it by changing quaternion in Matlab.

## Making art
You need 6 basic modules to make it all work:
- EKI interface;
- MOVEit + RViz;
- Environment frames broadcasting;
- Picture converter service;
- Trajectories drawing module;
- Starting trigger.

Let's launch them one by one.

### Eki interface
On KRC4 launch `eki_hw_interface`, on NUC in a new terminal do:
```
roslaunch kuka_eki_hw_interface test_hardware_interface.launch
```

### RViz and MOVEit
You need a planner and a simulation. Launch them with
```
roslaunch kuka_moveit_config demo.launch
```

### Environment
Tell the robot where the paint tin and the canvas are. Note that it is not necessary to launch `draw workspace` node, the `tf_broadcaster` shares the canvas size. It just doesn't show it in RViz.
```
rosrun fake_painter_enviroment_tf tf_broadcaster
```

### Pictures processor
All incoming pictures need to be processed. Launch the service.
```
rosrun picture_preprocessing TextConverter.py
```
When it receives the call, it processes a picture with a HP filter and creates a rosbag file with trajectories.

### Trajectories drawer
The mainest script here is the trajectories drawer itself. It waits for the picture, calls TextConverter service and draws the painting.
```
rosrun local_task_planner trajectory_drawing
```

## Send the robot a picture to draw
The robot listens to a specific ROS-topic where you need to pass the path to a desired picture. The picture should be square (width equals height) and made of lines. Send the path:
```
rostopic pub /run std_msgs/String "data: '<path_to_picture>'"
```
After that. Two windows pop up showing the contours and the tracks. Close them and see Gaka-Chu drawing. Watch out for safety and alwasy be ready to press emergency stop button.
When Gaka-Chu finishes his art, you can send another path to picture and painter repeats the whole process.
