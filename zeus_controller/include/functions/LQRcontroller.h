#ifndef LQRCONTROLLER_H_
#define LQRCONTROLLER_H_

#include "CommonCoord.h"
#include "CoordTrans.h"
#include<ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include<geometry_msgs/Vector3.h>
#include<carla_msgs/CarlaEgoVehicleState.h>
#include<carla_msgs/ControlTrackWaypoint.h>
#include<carla_msgs/ControlTrackWaypoints.h>
#include <eigen3/Eigen/Dense> 
#include<stdlib.h> 

using namespace std;
using namespace CoordLib;

namespace controller
{
    class LQRcontroller
    {
        public:
            LQRcontroller();
            ~LQRcontroller();
            void initialize(const vehicleParam* vehicle_param, const ENUCoord* vehicle_pos, const vector<double> &q, const double &r);
            

            void CalLQROutlineLst(); //calculate outline list of K matrix, using LQR
            void readLQROutlineLst();//read outline list of K martrix, using LQR,  for effectivement
            void PredictModule(); //predict next period vehicle position
            void CalControlError(); //calculate es, es_dot, efai, efai_dot
            void FeedforwardControl(); //feed forward control
            void update(vector<ENUCoord>::const_iterator way_begin, vector<ENUCoord>::const_iterator way_end);  //receive vehicle pos and waypoints every frame 
            void FinalControlAndOutput(); // intergration of lqr controller, execute after update() every frame
            double getFrontWheelAngle(); //get lqr control result out of this class
            void lqr_error_record();//record lqr controller error
            bool is_reach_end=false;
        protected:
            struct LQRoutline_lst
            {
                vector<double> vx_lst;
                vector<double> K0_lst;
                vector<double> K1_lst;
                vector<double> K2_lst;
                vector<double> K3_lst;
            };
            LQRoutline_lst lqr_outline_lst;

            vehicleParam vehicle_parameter;
            ENUCoord vehicle_pose;
            const vehicleParam* veh_para_ptr=&vehicle_parameter;
            const ENUCoord* vehicle_pos_ptr=& vehicle_pose;
            vector<ENUCoord>::const_iterator waypoints_begin;
            vector<ENUCoord>::const_iterator waypoints_end;
            vector<ENUCoord> track_waypoints;
            vector<double> s_lst;
            ENUCoord vehicle_predict_pos;
            ENUCoord lqr_proj_point;
            double predict_ts; 
            double front_wheel_angle;
            int match_index=0;
            const double discret_ts=0.01;
            const int state_size=4;
            bool is_recalK=true;

            Eigen::MatrixXd A_bar=Eigen::MatrixXd(state_size,state_size);
            Eigen::MatrixXd B_bar=Eigen::MatrixXd(state_size,1);
            Eigen::MatrixXd error=Eigen::MatrixXd(state_size,1);
            Eigen::MatrixXd K=Eigen::MatrixXd(1,state_size);
            Eigen::MatrixXd Q=Eigen::MatrixXd(state_size,state_size);
            Eigen::MatrixXd R=Eigen::MatrixXd(1,1);
            Eigen::MatrixXd P=Eigen::MatrixXd(state_size,state_size);
            Eigen::MatrixXd P_next=Eigen::MatrixXd(state_size,state_size);
            Eigen::MatrixXd delta_f=Eigen::MatrixXd(1,1);
            Eigen::MatrixXd u=Eigen::MatrixXd(1,1);

            //for data record
            //bool has_worte=false;
            vector<double> ed_lst;
            vector<double>ed_dot_lst;
            vector<double>e_fai_lst;
            vector<double>e_fai_dot_lst;
            vector<double>s_record_lst;
            vector<double>match_index_lst;
            vector<double>project_curv_lst;
            vector<double>ego_x;
            vector<double>ego_y;
    };
}
#endif
