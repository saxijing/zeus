# zeus READ ME
# 1. download directory
Install carla ROS Bridge in Ubuntu environment, and then download the above files to the "carla-ros-bridge/catkin_ws/src/ros-bridge" directory.
# 2. Introduce each ROS package
*carla_sensor_result: Python ROS node, used to obtain information in the Carla simulator, including ego vehicle's state, obstacle information, etc.
*zeus_common: C++ ROS package, implemented some class and function definitions shared by planner and controller modules, including the representation of ego vehicle and object, coordinate system transformation, etc.
*zeus_library:  C++ static library, which is the functions' implementation of zeus_common.
*zeus_config: It stores some configuration files to facilitate the reading of ROS nodes, which can avoid multiple compilations during debugging and test.
*zeus_launch: launch files for each module.
*zeus_display: C++ ROS node, used to receive and publish relevant information to display in rviz.
*zeus_calibration: C++ ROS node, used to calibrate velocity and acceleration in different threshold and brake, make threshold-brake table for longitudinal control.
*zeus_controller: C++ ROS node, including LQR controller based on vehicle dynamics for lateral control, and PID controller based on segmented control strategy for longitudinal control
*scenario_files: .xosc files, scenario files  used in simulation.
