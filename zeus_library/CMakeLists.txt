cmake_minimum_required(VERSION 3.10)

# 设置项目名称
project(zeus_library)

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 包含头文件的目录
include_directories(/home/saxijing/carla-ros-bridge/catkin_ws/src/ros-bridge/zeus_common/include)

# 将所有的源文件加入到可执行文件的构建中
# add_executable(CoordTrans src/CoordTrans.cpp src/main.cpp)
add_library(CoordTrans STATIC src/CoordTrans.cpp)

# 如果有多个源文件，可以将它们都加入到可执行文件中
# add_executable(MyExecutable main.cpp src/other.cpp)
