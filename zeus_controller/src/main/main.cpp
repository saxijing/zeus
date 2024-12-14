#include "controller.h"
#define NODE_NAME "controller"

using namespace std;
using namespace CoordLib;
using namespace controller;

int main(int argc, char** argv)
{
    //ROS_INFO(" %s starts, ros node name is " NODE_NAME, argv[0]);
    ros::init(argc, argv, "controller");
    ros::NodeHandle nh_ctrl;
    Controller ros_control(nh_ctrl);

    //ros::spin();
}

