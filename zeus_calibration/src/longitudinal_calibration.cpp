#include<iostream>
#include<ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include<geometry_msgs/Vector3.h>
#include <carla_msgs/CarlaEgoVehicleControl.h>
#include<carla_msgs/CarlaEgoVehicleState.h>

using namespace std;

static double vx;
static double vy;
static double yaw;
static double v;
static double ax;
static double ay;
static double a;
//static double x0,y0;
//static double x,y;

void recvCarlaPosCallback(const carla_msgs::CarlaEgoVehicleState &msg)
{
    //cout<<"come into callback"<<endl;
    yaw=msg.pose.theta;
    vx=msg.vel.x;
    vy=msg.vel.y;
    v=sqrt(vx*vx+vy*vy);
    ax=msg.accel.x;
    ay=msg.accel.y;
    a=ax*cos(yaw)+ay*sin(yaw);
    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "calibration");
    ros::NodeHandle nh_calib;
    carla_msgs::CarlaEgoVehicleControl msg;
    //carla pos pub frequence change to 100 hz for calibration, normally 50 hz
    ros::Subscriber sub=nh_calib.subscribe("/zeus/ego_vehicle_state", 10, &recvCarlaPosCallback);
    ros::Publisher pub=nh_calib.advertise<carla_msgs::CarlaEgoVehicleControl>("/carla/ego_vehicle/vehicle_control_cmd", 10, true);
    ros::Rate loop_rate(100);
    bool is_record=true;
    while (ros::ok())
    {
        msg.header.stamp=ros::Time::now();
        msg.steer=0.0;
        // if(is_record==false)
        // {
        //     msg.throttle=1.0;
        //     msg.brake=0.0;
        // }
        // else
        // {
        //     msg.throttle=0.0;
        //     msg.brake=1.0;
        // }
        
        msg.throttle=1.0;
        msg.brake=0.0;
        
        msg.hand_brake=false;
        msg.reverse=false;
        msg.gear=0;
        msg.manual_gear_shift=false;
        pub.publish(msg);
        ros::spinOnce();
        //if(v>=49) is_record=true;
        if((is_record==true)&&(v>=0)&&(v<=180/3.6))//
            cout<<v<<","<<a<<endl;//delete the first line
        loop_rate.sleep();
    }
}