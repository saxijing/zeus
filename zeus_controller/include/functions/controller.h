#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "LQRcontroller.h"
#include "PIDcontroller.h"
#include <carla_msgs/CarlaEgoVehicleControl.h>
//#include "PIDcontroller.h"

using namespace std;
using namespace CoordLib;

namespace controller
{
    class Controller
    {
        public:
            Controller(ros::NodeHandle &nh);
            ~Controller();

            //callback function
            void recvCarlaPosCallback(const carla_msgs::CarlaEgoVehicleState &msg);
            void recvControlWaypoints(const carla_msgs::ControlTrackWaypoints &msg);
            void readWaypoints();
            void readThrottleAndBrakeCalibTable();
            double lookupThrottleAndBrakeCalibTable(const double &vel, const double &accel);

            // main loop function
            void control(ros::NodeHandle &nh);

        protected:
            vehicleParam vehicle_param;
            ENUCoord vehicle_pos;
            int control_rate;
            //lateral variables
            vector<ENUCoord> track_waypoints;
            double steer_last;
            double steer;
            LQRcontroller lqrController;

            //longitudinal variables
            PIDcontroller speed_pid_controller;
            PIDcontroller accel_pid_controller;
            double target_vel;
            double current_vel;
            double target_accel;
            double table_value;
            vector<double>velocity_lst;
            vector<double>accel_lst;
            vector<vector<double>>thr_and_brake_lst;
            double speed_kp, speed_ki, speed_kd;
            double accel_kp, accel_ki, accel_kd;
            double discrete_time;

            ros::NodeHandle nh_;
            ros::Subscriber pos_sub;
            ros::Subscriber waypoints_sub;
            ros::Publisher controller_pub;
            ros::Time time_origin;
            ros::Time time_now;
            ros::Duration simulation_duration;

    };
}

#endif
