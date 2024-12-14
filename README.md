# zeus READ ME
# 1. download directory
Install carla ROS Bridge in Ubuntu environment, and then download the above files to the "carla-ros-bridge/catkin_ws/src/ros-bridge" directory.
# 2. Introduce each ROS package
* __carla_sensor_result:__ Python ROS node, used to obtain information in the Carla simulator, including ego vehicle's state, obstacle information, etc.
* __zeus_common:__ C++ ROS package, implemented some class and function definitions shared by planner and controller modules, including the representation of ego vehicle and object, coordinate system transformation, etc.
* __zeus_library:__  C++ static library, which is the functions' implementation of zeus_common.
* __zeus_config:__ It stores some configuration files to facilitate the reading of ROS nodes, which can avoid multiple compilations during debugging and test.
* __zeus_launch:__ launch files for each module.
* __zeus_display:__ C++ ROS node, used to receive and publish relevant information to display in rviz.
* __zeus_calibration:__ C++ ROS node, used to calibrate velocity and acceleration in different threshold and brake, make threshold-brake table for longitudinal control.
* __zeus_controller:__ C++ ROS node, including LQR controller based on vehicle dynamics for lateral control, and PID controller based on segmented control strategy for longitudinal control
* __scenario_files:__ .xosc files, scenario files  used in simulation.
