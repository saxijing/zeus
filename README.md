# zeus READ ME
# 0 Results

Carla simulation results of Zeus controller as following:

<div align=center> <img src="https://github.com/saxijing/zeus/blob/main/zeus_controller/data/display_materials/lqr_control_carla_sim.gif" width=700></div>

<p align="center">Fig.1 Zeus Controller's Carla simulation results</p>

<div align=center> <img src="https://github.com/saxijing/zeus/blob/main/zeus_controller/data/display_materials/lqr_control_trajectory.png" width=900></div>

<p align="center">Fig.2 Comparison diagram of ego vehicle trajectory and reference trajectory</p>

Combining Fig.1 and Fig.2, the ego vehicle controlled by the **Zeus controller** basically drives along the target trajectory. There is only a significant error in the section where the curvature of the target trajectory changes greatly, but it is still within the allowable range. The following are the specific error values.

<div align=center> <img src="https://github.com/saxijing/zeus/blob/main/zeus_controller/data/display_materials/lqr_control_error_a%3D1.62.png" width=1500></div>

<p align="center">Fig.3 Zeus controller error</p>

<div align=center> <img src="https://github.com/saxijing/zeus/blob/main/zeus_controller/data/display_materials/lqr_control_road_curv.png" width=600></div>

<p align="center">Fig.4 Reference trajectory curvature</p>

Combining Fig.3 and Fig.4, simulation conclusions are as following:

(1) On road segments with **a curvature of less than $0.1 m^{-1}$**, the **lateral distance errors are within centimeters**, and the **yaw angle errors are within $0.1 ^ {\circ}$**;

(2) On road segments with **a curvature greater than $0.1 m^{-1}$ or where the curvature changes abruptly**, the **lateral distance errors range from $-1 \sim 1.5m$**, and the **yaw angle errors are within $\pm2 ^{\circ}$**.

**Tab.1 Average Errors of Coordinate Transformation**

|distance error(m) |yaw_angle error($^。$) |curvature error($m^{-1}$) |velocity error(m/s) |acceleration error(%) |
|:------:|:------:|:------:|:------:|:------:|
|0.2|7.4|0|$9 \times 10 ^{-4}$|58.3|

Above Table 1 presents the average errors (across 12 test cases) of the Cartesian coordinate values $(x, y, yaw, curv, v, a)$ after a two -step **Cartesian -> Frenet -> Cartesian** transformation comapred to the original values.
The results show that the **distance error, yaw error, curvature error, and velocity error all remain within acceptable tolerances**. However, **acceleration transformation is significantly affected by road curvature**, leading to unacceptably large errors at a part of sampling points where the **raidus of curvature is below 60m**. Consequently, the accuracy rate of acceleration transformation is **58.3%**.

# 1 Local Path of Repository
Install carla ROS Bridge in Ubuntu environment, and then clone the repository to the "carla-ros-bridge/catkin_ws/src/ros-bridge" directory.

# 2 Introduction to Zeus Project

## 2.1 Introduction to each ROS package

* __carla_sensor_result:__ Python ROS node, used to obtain information in the Carla simulator, including ego vehicle's state, obstacle information, etc.
* __zeus_common:__ C++ ROS package, implemented some class and function definitions shared by planner and controller modules, including the representation of ego vehicle and object, coordinate system transformation, etc.
* __zeus_library:__  C++ static library, which is the functions' implementation of zeus_common.
* __zeus_config:__ It stores some configuration files to facilitate the reading of ROS nodes, which can avoid multiple compilations during debugging and test.
* __zeus_launch:__ launch files for each module.
* __zeus_display:__ C++ ROS node, used to receive and publish relevant information to display in rviz.
* __zeus_calibration:__ C++ ROS node, used to calibrate velocity and acceleration in different threshold and brake, make threshold-brake table for longitudinal control. The calibration results are stored in this directory "zeus/zeus_controller/data/pid_control".
* __zeus_controller:__ C++ ROS node, including LQR controller based on vehicle dynamics for lateral control, and PID controller based on segmented control strategy for longitudinal control
* __scenario_files:__ .xosc files, scenario files  used in simulation.

## 2.2 Project Architecture

In order to be closer to real-world vehicle operating environment, this project employs C++ code programming and utilizes ROS for inter process communication, relying on Carla simulator to complete verification. The overall architecture and data flow diagram of the project are illustrated in Fig.5.The Carla simulator operates the scenario file to provide simulation environment, while the library named carla-ros-bridge connects the Carla simulation environment with ROS nodes.

The **carla_sensor_result node** simulates sensor acquisition of the ego vehicle state and surrounding environment information, publishing these as ROS messages. The **zeus_controller node**, serving as the core module of this project, subscribes ego vehicle state, processes it through the **LQR Controller** and **PID Controller**, and publishes control signals including throttle, brake, and steer. Upon subscribing these control signals, the carla-ros-bridge updates the ego vehicle pose within Carla, thereby completing the control loop.

To enhance the zeus_controller's responsiveness, certain computations -- such as the LQR feedback matrix K and throttle-brake calibration table -- are precomputed offline. During real-time operation, these values are retrived via table lookups. Additionally, commonly used functions shared between controller and planner -- such as coordinate transformations between Frenet and Cartesian -- are implemented as static libraries in **zeus_library** directory to enable multi-module accessibility. Please refer to Fig.5 for more details.

<div align=center> <img src="https://github.com/saxijing/zeus/blob/main/zeus_controller/data/display_materials/zeus_architecture.png" width=800></div>

<p align="center">Fig.5 Zeus Project Architecture</p>

# 3 Others

## 3.1 Command line to be executed

roslaunch carla_ros_bridge carla_ros_bridge.launch

python scenario_runner.py --openscenario /home/saxijing/carla-ros-bridge/catkin_ws/data/reference_point/zeus_calibration_06.xosc

roslaunch carla_sensor_result carla_sensor_result.launch

rosrun zeus_calibration calibrater>calibration.txt

roslaunch zeus_launch controller.launch>launch_out.txt

catkin_make -DCATKIN_WHITELIST_PACKAGES="zeus_controller"

rivz(optional):

rosrun zeus_display displayLane

rosrun tf static_transform_publisher 0 0 0 0 0 0 map world 10

## 3.2 Scene file description

LQR test scenario: zeus_control_test07flat.xosc

Threshold -brake calibration/PID test scenario: zeus_calibration_06.xosc

Overoll test and display: zeus_control_test01.xosc
