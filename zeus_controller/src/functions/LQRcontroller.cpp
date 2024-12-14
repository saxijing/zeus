#include "LQRcontroller.h"

using namespace std;
using namespace CoordLib;
using namespace Eigen;

namespace controller
{
    
    LQRcontroller::LQRcontroller()
    {
        //ROS_INFO("construct lqr controller ! ");
    }

    void LQRcontroller::initialize(const vehicleParam* vehicle_param, const ENUCoord* vehicle_pos, const vector<double> &q, const double &r)
    {
        //ROS_INFO("Initialize LQR controller...");
        is_recalK=true;
        veh_para_ptr=vehicle_param;
        vehicle_pos_ptr=vehicle_pos;
        //ROS_INFO("lqr data success.");
        for(int i=0; i<Q.rows(); i++)
        {
            for(int j=0; j<Q.cols();j++)
            {
                if(i==j)
                    Q(i,j)=q[i];
                else
                    Q(i,j)=0;
            }
        }
        R(0,0)=r;
        if(is_recalK)
            CalLQROutlineLst();
        else
            readLQROutlineLst();
    }

    LQRcontroller::~LQRcontroller()
    {
        //ROS_INFO("destroyed lqr controller~");
    }

    void LQRcontroller::CalLQROutlineLst()
    {
        //ROS_INFO(" calculating LQR parameters outline table.....");
        lqr_outline_lst.vx_lst.clear();
        lqr_outline_lst.K0_lst.clear();
        lqr_outline_lst.K1_lst.clear();
        lqr_outline_lst.K2_lst.clear();
        lqr_outline_lst.K3_lst.clear();
        double a11, a12, a13, a31, a32, a33;
        double b1, b3;
        double vx_mat;
        int vx_int;
        double a=veh_para_ptr->center2frontaxies;
        double b=veh_para_ptr->center2rearaxies;
        double cf=veh_para_ptr->front_wheel_lat_stiff;
        double cr=veh_para_ptr->rear_wheel_lat_stiff;
        double m=veh_para_ptr->vehicle_mass;
        double I=veh_para_ptr->inertial_moment;
        Eigen::MatrixXd K_temp(1,state_size);
        Eigen::MatrixXd A=MatrixXd::Zero(state_size, state_size);
        Eigen::MatrixXd B=MatrixXd::Zero(state_size, 1);
        Eigen::MatrixXd I_matrix=MatrixXd::Identity(state_size, state_size);

        int counter=0;
        int counter1=0;
        for(vx_int=1; vx_int<3400;vx_int++) //42mps=150kph
        {
            counter1=0;
            P=Q;
            vx_mat=vx_int/100.0;
            a11=(cf+cr)/(m*vx_mat);
            a12=-(cf+cr)/m;
            a13=(a*cf-b*cr)/(m*vx_mat);
            a31=(a*cf-b*cr)/(I*vx_mat);
            a32=-(a*cf-b*cr)/I;
            a33=(a*a*cf+b*b*cr)/(I*vx_mat);
            b1=-cf/m;
            b3=-a*cf/I;

            A(0,1)=1.0;
            A(1,1)=a11;
            A(1,2)=a12;
            A(1,3)=a13;
            A(2,3)=1.0;
            A(3,1)=a31;
            A(3,2)=a32;
            A(3,3)=a33;
            A_bar=(I_matrix-A*discret_ts*0.5).inverse()*(I_matrix-A*discret_ts*0.5);
            
            B(1,0)=b1;
            B(3,0)=b3;
            B_bar=B*discret_ts;
    
            for(int i=0; i<1500; i++)
            {
                P_next= A_bar.transpose() * P * A_bar - A_bar.transpose() * P * B_bar * (R + B_bar.transpose() * P * B_bar).inverse() * B_bar.transpose() * P * A_bar + Q;
                if ((P_next - P).norm() < 1e-6)
                    break;
                P=P_next;
                counter1++;
            }

            K_temp=(R + B_bar.transpose() * P * B_bar).inverse() * B_bar.transpose() * P * A_bar;
            
            lqr_outline_lst.vx_lst.push_back(vx_mat);
            lqr_outline_lst.K0_lst.push_back(K_temp(0,0));
            lqr_outline_lst.K1_lst.push_back(K_temp(0,1));
            lqr_outline_lst.K2_lst.push_back(K_temp(0,2));
            lqr_outline_lst.K3_lst.push_back(K_temp(0,3));
            counter++;
            //cout<<"vx_mat="<<vx_mat<<", k0="<<K_temp(0,0)<<", k1="<<K_temp(0,1)<<", k2="<<K_temp(0,2)<<", k3="<<K_temp(0,3)<<endl;
            cout<<vx_mat<<","<<K_temp(0,0)<<","<<K_temp(0,1)<<","<<K_temp(0,2)<<","<<K_temp(0,3)<<endl;
        }
        //ROS_INFO("LQR outline list finished! ");
        //double end_time=ros::Time::now().toSec();
        //cout<<"Total cost "<<(end_time-start_time)/60.0<<" min."<<endl;
    }

    void LQRcontroller::readLQROutlineLst()
    {
        lqr_outline_lst.vx_lst.clear();
        lqr_outline_lst.K0_lst.clear();
        lqr_outline_lst.K1_lst.clear();
        lqr_outline_lst.K2_lst.clear();
        lqr_outline_lst.K3_lst.clear();
        ifstream filew("/home/saxijing/carla-ros-bridge/catkin_ws/src/ros-bridge/zeus_controller/data/lqr_outline_lst.csv");
        if(!filew.is_open())
        {
            cerr<<"无法打开文件"<<endl;
            return;
        }
        string linew;
        getline(filew,linew);
        while(getline(filew,linew))
        {
            stringstream ss(linew);
            string itemw;

            for(int col=0; getline(ss, itemw, ','); ++col)
            {
                try
                {
                    switch(col)
                    {
                        case 0:
                            lqr_outline_lst.vx_lst.push_back(stod(itemw));
                            break;
                        case 1:
                            lqr_outline_lst.K0_lst.push_back(stod(itemw));
                            break;
                        case 2:
                            lqr_outline_lst.K1_lst.push_back(stod(itemw));
                            break;
                        case 3:
                            lqr_outline_lst.K2_lst.push_back(stod(itemw));
                            break;
                        case 4:
                            lqr_outline_lst.K3_lst.push_back(stod(itemw));
                            break;
                    }
                }
                catch(const invalid_argument &e)
                {    cerr<<"转换错误："<<e.what()<<endl;}
                catch(const out_of_range &e)
                {    cerr<<"值超出范围："<<e.what()<<endl;}
            }
        }
        filew.close();
    }

    void LQRcontroller::PredictModule()
    {
        predict_ts=0.1;
        double vx=vehicle_pos_ptr->velocity_x*cos(vehicle_pos_ptr->yaw)+vehicle_pos_ptr->velocity_y*sin(vehicle_pos_ptr->yaw);
        double vy=-vehicle_pos_ptr->velocity_x*sin(vehicle_pos_ptr->yaw)+vehicle_pos_ptr->velocity_y*cos(vehicle_pos_ptr->yaw);

        vehicle_predict_pos.x=vehicle_pos_ptr->x+vx*predict_ts*cos(vehicle_pos_ptr->yaw)-vy*predict_ts*sin(vehicle_pos_ptr->yaw);
        vehicle_predict_pos.y=vehicle_pos_ptr->y+vy*predict_ts*cos(vehicle_pos_ptr->yaw)+vx*predict_ts*sin(vehicle_pos_ptr->yaw);
        vehicle_predict_pos.z=0.0;

        vehicle_predict_pos.yaw=vehicle_pos_ptr->yaw+vehicle_pos_ptr->yaw_rate*predict_ts;
        vehicle_predict_pos.pitch=0.0;
        vehicle_predict_pos.roll=0.0;

        vehicle_predict_pos.velocity_x=vehicle_pos_ptr->velocity_x;
        vehicle_predict_pos.velocity_y=vehicle_pos_ptr->velocity_y;
        vehicle_predict_pos.velocity_z=0.0;
        vehicle_predict_pos.velocity=vehicle_pos_ptr->velocity;

        vehicle_predict_pos.accel_x=vehicle_pos_ptr->accel_x;
        vehicle_predict_pos.accel_y=vehicle_pos_ptr->accel_y;
        vehicle_predict_pos.accel_z=0.0;
        vehicle_predict_pos.accel=vehicle_pos_ptr->accel;

        vehicle_predict_pos.curvature=vehicle_pos_ptr->curvature;
        vehicle_predict_pos.yaw_rate=vehicle_pos_ptr->yaw_rate;
        vehicle_predict_pos.velocity_angle=vehicle_pos_ptr->velocity_angle;
    }

    void LQRcontroller::CalControlError()
    {
        //ENUCoord lqr_proj_point;
        vector<double>error_temp;
        double s, dkr;
        //match_index=0;
        //cout<<"track_waypoints length="<<track_waypoints.size()<<"; s_lst length="<<s_lst.size()<<endl;
        find_c2f_project_point_and_control_error(*vehicle_pos_ptr, lqr_proj_point, track_waypoints, s_lst, match_index, s, dkr, error_temp);
        //radian_unify(error_temp[2]);
        error(0,0)=error_temp[0];
        error(1,0)=error_temp[1];
        error(2,0)=error_temp[2];
        error(3,0)=error_temp[3];
        radian_unify(error(2,0));
        cout<<"s,match_i,ed,ed_dot,efai,efai_dot,project_curv,ego_x,ego_y:";
        cout<<s<<","<<match_index<<","<<error(0,0)<<","<<error(1,0)<<","<<error(2,0)<<","<<error(3,0)<<","<<lqr_proj_point.curvature<<","<<vehicle_pos_ptr->x<<","<<vehicle_pos_ptr->y<<endl;
        // cout<<"error_vector={ "<<error_temp[0]<<", "<<error_temp[1]<<", "<<error_temp[2]<<", "<<error_temp[3]<<" }"<<endl;
        // cout<<"error_matrix={ "<<error(0,0)<<", "<<error(1,0)<<", "<<error(2,0)<<", "<<error(3,0)<<" }"<<endl;
        // cout<<"match_index="<<match_index<<endl;
        // cout<<"project point: x="<<lqr_proj_point.x<<", y="<<lqr_proj_point.y<<", hdg="<<lqr_proj_point.velocity_angle<<", curv="<<lqr_proj_point.curvature<<endl;
        // cout<<"s_record_length="<<s_record_lst.size()<<endl;
        if(match_index>=1840&&s_record_lst.size()>=1000)
            is_reach_end=true;
    }

    void LQRcontroller::FeedforwardControl()
    {
        double a=veh_para_ptr->center2frontaxies;
        double b=veh_para_ptr->center2rearaxies;
        double cf=veh_para_ptr->front_wheel_lat_stiff;
        double cr=veh_para_ptr->rear_wheel_lat_stiff;
        double m=veh_para_ptr->vehicle_mass;
        /* using current pos*/
        /* using predicted pos*/
        double fai=vehicle_pos_ptr->yaw;
        double vx=vehicle_pos_ptr->velocity_x*cos(fai)+vehicle_pos_ptr->velocity_y*sin(fai); //feed forward module use actual vx instead of predicted vx
        double vy=-vehicle_pos_ptr->velocity_x*sin(fai)+vehicle_pos_ptr->velocity_y*cos(fai);

        delta_f(0,0)=track_waypoints[match_index].curvature*(a+b-b*K(0,2)-(m*vx*vx/(a+b))*(b/cf+a*K(0,2)/cr-a/cr));
        
        //cout<<"delta_f="<<delta_f(0,0)<<endl;
        //cout<<"vx="<<vx<<", vy="<<vy<<endl;
    }

    void LQRcontroller::update(vector<ENUCoord>::const_iterator way_begin, vector<ENUCoord>::const_iterator way_end)
    {
        int counter=0;
        ENUCoord point;
        s_lst.push_back(0.0);
        for(auto it=way_begin; it!=way_end; it++)
        {
            point.x=(*it).x;
            point.y=(*it).y;
            point.yaw=(*it).yaw;
            point.velocity_angle=(*it).velocity_angle;
            point.velocity=(*it).velocity;
            point.velocity_x=(*it).velocity_x;
            point.velocity_y=(*it).velocity_y;
            point.accel=(*it).accel;
            point.accel_x=(*it).accel_x;
            point.accel_y=(*it).accel_y;
            point.curvature=(*it).curvature;
            track_waypoints.push_back(point);
            if(counter==0)
            {
                counter++;
                continue;
            }
            double dist=sqrt(pow((*it).x-(*(it-1)).x,2) + pow((*it).y-(*(it-1)).y,2));
            s_lst.push_back(dist+s_lst[counter-1]);
            counter++;
        }
        //cout<<"counter="<<counter<<endl;
    }

    void LQRcontroller::FinalControlAndOutput()
    {
        PredictModule();
        CalControlError();

        // look up LQRoutline_lst, interpolation for K matrix
        int vx_match_index=0;
        double percent=0.0;
        double fai=vehicle_pos_ptr->yaw;
        double veh_vx=vehicle_pos_ptr->velocity_x*cos(fai)+vehicle_pos_ptr->velocity_y*sin(fai);
        if(veh_vx<0.03) veh_vx=0.03;
        for(int i=0; i<lqr_outline_lst.vx_lst.size()-1;i++)
        {
            if(veh_vx>=lqr_outline_lst.vx_lst[i] && veh_vx<lqr_outline_lst.vx_lst[i+1])
            {
                vx_match_index=i;
                break;
            }
        }
        //percent=(veh_vx-lqr_outline_lst.vx_lst[vx_match_index])/(lqr_outline_lst.vx_lst[vx_match_index+1]-lqr_outline_lst.vx_lst[vx_match_index]);
        // double k0=lqr_outline_lst.K0_lst[vx_match_index]+percent*(lqr_outline_lst.K0_lst[vx_match_index+1]-lqr_outline_lst.K0_lst[vx_match_index]);
        // double k1=lqr_outline_lst.K1_lst[vx_match_index]+percent*(lqr_outline_lst.K1_lst[vx_match_index+1]-lqr_outline_lst.K1_lst[vx_match_index]);
        // double k2=lqr_outline_lst.K2_lst[vx_match_index]+percent*(lqr_outline_lst.K2_lst[vx_match_index+1]-lqr_outline_lst.K2_lst[vx_match_index]);
        // double k3=lqr_outline_lst.K3_lst[vx_match_index]+percent*(lqr_outline_lst.K3_lst[vx_match_index+1]-lqr_outline_lst.K3_lst[vx_match_index]);
        double k0=lqr_outline_lst.K0_lst[vx_match_index];
        double k1=lqr_outline_lst.K1_lst[vx_match_index];
        double k2=lqr_outline_lst.K2_lst[vx_match_index];
        double k3=lqr_outline_lst.K3_lst[vx_match_index];
        K(0,0)=k0;
        K(0,1)=k1;
        K(0,2)=k2;
        K(0,3)=k3;
        //cout<<"K_matrix={ "<<K(0,0)<<", "<<K(0,1)<<", "<<K(0,2)<<", "<<K(0,3)<<" }"<<endl;

        FeedforwardControl();
        u=-K*error+delta_f;
        front_wheel_angle=u(0,0);
        //cout<<"front_wheel_angle(rad): "<<front_wheel_angle<<endl;
        //cout<<"front_wheel_angle(degree): "<<front_wheel_angle*180/M_PI<<endl;
    }

    double LQRcontroller::getFrontWheelAngle()
    {
        track_waypoints.clear();
        s_lst.clear();
        return front_wheel_angle;
    }

}