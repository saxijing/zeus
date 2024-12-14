#include <ros/ros.h>
#include<iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include<geometry_msgs/Pose.h>
#include<geometry_msgs/PoseStamped.h>
#include<nav_msgs/Path.h>

using namespace std;
int main(int argc, char** argv)
{
    ros::init(argc, argv, "visualize_lane");
    ros::NodeHandle nh;
    ros::Publisher refer_pub=nh.advertise<nav_msgs::Path>("/rviz/global_line/reference_line",2);
    ros::Publisher center_pub=nh.advertise<nav_msgs::Path>("/rviz/global_line/center_line",2);
    ros::Publisher left_pub=nh.advertise<nav_msgs::Path>("/rviz/global_line/left_edge",2);
    ros::Publisher right_pub=nh.advertise<nav_msgs::Path>("/rviz/global_line/right_edge",2);

    ros::Rate rate(1);

    nav_msgs::Path referLine_msg;
    nav_msgs::Path leftEdge_msg;
    nav_msgs::Path rightEdge_msg;
    nav_msgs::Path center_msg;

    geometry_msgs::PoseStamped point_refer;
    geometry_msgs::PoseStamped point_center;
    geometry_msgs::PoseStamped point_left;
    geometry_msgs::PoseStamped point_right;

    double refer_yaw, center_yaw, left_yaw, right_yaw;

    ifstream filew("/home/saxijing/carla-ros-bridge/catkin_ws/data/reference_point/Town07flat_waypoints_and_roadEdge_map.csv");
    if(!filew.is_open())
    {
        cerr<<"无法打开文件"<<endl;
        return 0;
    }
    string linew;
    getline(filew,linew);
    int row=0;
    while(getline(filew,linew))
    {
        stringstream sss(linew); // 用于将行内容分割为单独的字段
        string itemw;

        for(int col=0; getline(sss, itemw, ','); ++col)
        {
            try
            {
                switch(col)
                {
                    //left
                    case 1:
                        point_left.pose.position.x=stod(itemw);
                        break;
                    case 2:
                        point_left.pose.position.y=stod(itemw);
                        point_left.pose.position.z=0.0;
                        break;
                    case 3:
                        left_yaw=stod(itemw);
                        point_left.pose.orientation.w=cos(left_yaw/2);
                        point_left.pose.orientation.z=sin(left_yaw/2);
                        point_left.pose.orientation.y=0.0;
                        point_left.pose.orientation.x=0.0;
                        break;
                    
                    //center
                    case 7:
                        point_center.pose.position.x=stod(itemw);
                        break;
                    case 8:
                        point_center.pose.position.y=stod(itemw);
                        point_center.pose.position.z=0.0;
                        break;
                    case 9:
                        center_yaw=stod(itemw);
                        point_center.pose.orientation.w=cos(center_yaw/2);
                        point_center.pose.orientation.z=sin(center_yaw/2);
                        point_center.pose.orientation.y=0.0;
                        point_center.pose.orientation.x=0.0;
                        break;

                    //refer
                    case 13:
                        point_refer.pose.position.x=stod(itemw);
                        break;
                    case 14:
                        point_refer.pose.position.y=stod(itemw);
                        point_refer.pose.position.z=0.0;
                        break;
                    case 15:
                        refer_yaw=stod(itemw);
                        point_refer.pose.orientation.w=cos(refer_yaw/2);
                        point_refer.pose.orientation.z=sin(refer_yaw/2);
                        point_refer.pose.orientation.y=0.0;
                        point_refer.pose.orientation.x=0.0;
                        break;
                    
                    //right
                    case 19:
                        point_right.pose.position.x=stod(itemw);
                        break;
                    case 20:
                        point_right.pose.position.y=stod(itemw);
                        point_right.pose.position.z=0.0;
                        break;
                    case 21:
                        right_yaw=stod(itemw);
                        point_right.pose.orientation.w=cos(right_yaw/2);
                        point_right.pose.orientation.z=sin(right_yaw/2);
                        point_right.pose.orientation.y=0.0;
                        point_right.pose.orientation.x=0.0;
                        break;
                }
            }
            catch(const invalid_argument &e)
            {    cerr<<"转换错误："<<e.what()<<endl;}
            catch(const out_of_range &e)
            {    cerr<<"值超出范围："<<e.what()<<endl;}

        }
        
        referLine_msg.poses.push_back(point_refer);
        leftEdge_msg.poses.push_back(point_left);
        center_msg.poses.push_back(point_center);
        rightEdge_msg.poses.push_back(point_right);
        row++;
    }
    filew.close();

    // publish path
    while(ros::ok())
    {
        referLine_msg.header.stamp=ros::Time::now();
        center_msg.header.stamp=ros::Time::now();
        leftEdge_msg.header.stamp=ros::Time::now();
        rightEdge_msg.header.stamp=ros::Time::now();

        referLine_msg.header.frame_id="world";
        center_msg.header.frame_id="world";
        leftEdge_msg.header.frame_id="world";
        rightEdge_msg.header.frame_id="world";

        refer_pub.publish(referLine_msg);
        center_pub.publish(center_msg);
        left_pub.publish(leftEdge_msg);
        right_pub.publish(rightEdge_msg);
        rate.sleep();
    }
    return 0;
}
