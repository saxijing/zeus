#ifndef PIDCONTROLLER_H_
#define PIDCONTROLLER_H_

#include "CommonCoord.h"
#include "CoordTrans.h"
#include<ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include<geometry_msgs/Vector3.h>
#include<carla_msgs/CarlaEgoVehicleState.h>
#include<carla_msgs/ControlTrackWaypoint.h>
#include<carla_msgs/ControlTrackWaypoints.h>
//#include <eigen3/Eigen/Dense> 
#include<stdlib.h> 

using namespace std;
using namespace CoordLib;

namespace controller
{
    class PIDcontroller
    {
        public:
            PIDcontroller();
            void initialize(const double &kp_, const double &ki_, const double &kd_, const double &dt_);
            ~PIDcontroller();
            double pid_control(const double &err);
            void Reset();
        protected:
            double kp;
            double ki;
            double kd;
            double dt;
            double error;
            double previous_error;
            double previous_output;
            double integral;
            double differential;
            double error_sum;
            double error_diff;
            bool first_hit;
    };
}
#endif
