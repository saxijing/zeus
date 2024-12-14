#include "controller.h"

using namespace std;
using namespace CoordLib;

namespace controller
{
    Controller::Controller(ros::NodeHandle &nh): lqrController(), speed_pid_controller(), accel_pid_controller(), nh_(nh)
    {
        //ROS_INFO("Start to load vehicle parameters......");
        vector<double>Q_tri;
        double q0,q1,q2,q3;
        double R_tri;
        double wheelbase;

        nh.getParam("/controller/vehicle_mass",vehicle_param.vehicle_mass);
        nh.getParam("/controller/wheelbase",wheelbase);
        nh.getParam("/controller/latStiffness_front",vehicle_param.front_wheel_lat_stiff);
        nh.getParam("/controller/latStiffness_rear",vehicle_param.rear_wheel_lat_stiff);
        nh.getParam("/controller/front_wheel_max_angle",vehicle_param.front_wheel_max_angle);
        nh.getParam("/controller/rear_wheel_max_angle",vehicle_param.rear_wheel_max_angle);
        nh.getParam("/controller/LQRcontroller/q1",q0);
        nh.getParam("/controller/LQRcontroller/q2",q1);
        nh.getParam("/controller/LQRcontroller/q3",q2);
        nh.getParam("/controller/LQRcontroller/q4",q3);
        nh.getParam("/controller/LQRcontroller/r",R_tri);
        nh.getParam("/controller/control_rate",control_rate);

        nh.getParam("/controller/speed_pid/kp", speed_kp);
        nh.getParam("/controller/speed_pid/ki", speed_ki);
        nh.getParam("/controller/speed_pid/kd", speed_kd);
        nh.getParam("/controller/accel_pid/kp", accel_kp);
        nh.getParam("/controller/accel_pid/ki", accel_ki);
        nh.getParam("/controller/accel_pid/kd", accel_kd);
        nh.getParam("/controller/pid/dt", discrete_time);

        /*test a=1.62, b=1.38*/
        vehicle_param.center2frontaxies=1.62;
        vehicle_param.center2rearaxies=wheelbase-vehicle_param.center2frontaxies;
        double ma=vehicle_param.vehicle_mass*vehicle_param.center2rearaxies/wheelbase;
        double mb=vehicle_param.vehicle_mass*vehicle_param.center2frontaxies/wheelbase;

        /*a=b=2.852/2*/
        // double ma=vehicle_param.vehicle_mass/2;
        // double mb=vehicle_param.vehicle_mass/2;
        // vehicle_param.center2frontaxies=wheelbase*(1.0-ma/(ma+mb));
        // vehicle_param.center2rearaxies=wheelbase*(1.0-mb/(ma+mb));
        
        vehicle_param.inertial_moment=vehicle_param.center2frontaxies*vehicle_param.center2frontaxies*ma+vehicle_param.center2rearaxies*vehicle_param.center2rearaxies*mb;
        
        Q_tri={q0,q1,q2,q3};
        //ROS_INFO("ros node handle register.....");
        pos_sub=nh.subscribe("/zeus/ego_vehicle_state", 10, &Controller::recvCarlaPosCallback, this);
        //waypoints_sub=nh.subscribe("/zeus/ego_vehicle/planned_path",1, &Controller::recvControlWaypoints, this);
        controller_pub=nh.advertise<carla_msgs::CarlaEgoVehicleControl>("/carla/ego_vehicle/vehicle_control_cmd", 10, true);
        readWaypoints();
        readThrottleAndBrakeCalibTable();
        //lqrController.initialize(&vehicle_param, &vehicle_pos, Q_tri, R_tri);
        speed_pid_controller.initialize(speed_kp, speed_ki, speed_kd, discrete_time);
        accel_pid_controller.initialize(accel_kp, accel_ki, accel_kd, discrete_time);

        control(nh_);
    }

    Controller::~Controller()
    {
        lqrController.~LQRcontroller();
        speed_pid_controller.~PIDcontroller();
        accel_pid_controller.~PIDcontroller();
    }

    void Controller::recvCarlaPosCallback(const carla_msgs::CarlaEgoVehicleState &msg)
    {
        vehicle_pos.x=msg.pose.x;
        vehicle_pos.y=msg.pose.y;
        vehicle_pos.z=0;

        vehicle_pos.velocity_x=msg.vel.x;
        vehicle_pos.velocity_y=msg.vel.y;
        vehicle_pos.velocity_z=0;
        vehicle_pos.velocity=sqrt(pow(vehicle_pos.velocity_x,2)+pow(vehicle_pos.velocity_y,2));

        vehicle_pos.accel_x=msg.accel.x;
        vehicle_pos.accel_y=msg.accel.y;
        vehicle_pos.accel_z=0;
        vehicle_pos.accel=sqrt(pow(vehicle_pos.accel_x,2)+pow(vehicle_pos.velocity_y,2));

        vehicle_pos.yaw=msg.pose.theta;
        vehicle_pos.pitch=0,
        vehicle_pos.roll=0;

        vehicle_pos.velocity_angle=msg.theta;//rad
        //radian_unify(vehicle_pos.velocity_angle);
        
        vehicle_pos.yaw_rate=msg.vel.theta;
        vehicle_pos.pitch_rate=0;
        vehicle_pos.roll_rate=0;

        vehicle_pos.curvature=0;
    }

    void Controller::recvControlWaypoints(const carla_msgs::ControlTrackWaypoints &msg)
    {
        ENUCoord point;
        for(int i=0; i<msg.waypoints.size();i++)
        {
            point.x=msg.waypoints[i].pose.x;
            point.y=msg.waypoints[i].pose.y;
            point.z=0;
            
            point.velocity=msg.waypoints[i].vel.z;
            point.velocity_x=msg.waypoints[i].vel.x;
            point.velocity_y=msg.waypoints[i].vel.y;
            point.velocity_z=0;

            point.accel=msg.waypoints[i].accel.z;
            point.accel_x=msg.waypoints[i].accel.x;
            point.accel_y=msg.waypoints[i].accel.y;
            point.accel_z=0;

            point.yaw=0;
            point.pitch=0;
            point.roll=0;

            point.velocity_angle=msg.waypoints[i].pose.z;
            point.curvature=msg.waypoints[i].curv;
            track_waypoints.push_back(point);
        }
    }

    void Controller::control(ros::NodeHandle &nh)
    {
        //ROS_INFO("Come into main loop of controller......");
        double lqr_output_angle;
        carla_msgs::CarlaEgoVehicleControl msg;
        ros::Rate loop_rate(control_rate);
        steer_last=0.0;
        int counter=0;
        time_origin=ros::Time::now();
        while(ros::ok())
        {
            ros::spinOnce();
            //lqrController.update(track_waypoints.begin(), track_waypoints.end());
            // cout<<"vehicle pos: x="<<vehicle_pos.x<<", y="<<vehicle_pos.y<<", z="<<vehicle_pos.z<<", yaw="<<vehicle_pos.yaw<<endl;
            // cout<<"velocity="<<vehicle_pos.velocity<<", x="<<vehicle_pos.velocity_x<<", y="<<vehicle_pos.velocity_y<<", velocity_angle="<<vehicle_pos.velocity_angle<<endl;
            // cout<<"acceleration="<<vehicle_pos.accel<<", x="<<vehicle_pos.accel_x<<", y="<<vehicle_pos.accel_y<<endl;
            // cout<<"yaw_rate="<<vehicle_pos.yaw_rate<<endl;
            // lqrController.FinalControlAndOutput();
            // lqr_output_angle=lqrController.getFrontWheelAngle();
            // lqr_output_angle=lqr_output_angle*180/M_PI;

            msg.header.stamp=ros::Time::now();
            /*lateral controller 
            if(counter%5<=3)
                msg.throttle=0.38;
            else
                msg.throttle=0.0;
            msg.brake=0.0;
            msg.hand_brake=false;
            msg.reverse=false;
            msg.gear=0;
            msg.manual_gear_shift=false;
            steer=-lqr_output_angle/vehicle_param.front_wheel_max_angle; 
            if (steer>1.0)
                steer=1.0;
            if (steer<-1.0)
                steer=-1.0;
            if(fabs(steer)<1e-5)
                steer=0.0;

            if(isnan(steer))
                msg.steer=steer_last;
            else
            {
                msg.steer=steer;
                steer_last=steer;
            }
            controller_pub.publish(msg);
            counter++;
            */
            time_now=ros::Time::now();
            if(counter==1) time_origin=time_now;
            simulation_duration=time_now-time_origin;
            target_vel=0.05;
            current_vel=sqrt(vehicle_pos.velocity_x*vehicle_pos.velocity_x+vehicle_pos.velocity_y*vehicle_pos.velocity_y);
            target_accel=accel_pid_controller.pid_control(target_vel-current_vel);
            cout<<simulation_duration<<","<<current_vel<<","<<target_vel<<endl;
            //cout<<target_accel<<endl;
            if(target_accel<=-8.0)
                target_accel=-8.0;
            else if(target_accel>=6.0)
                target_accel=6.0;

            table_value=lookupThrottleAndBrakeCalibTable(current_vel, target_accel);
            // cout<<"vel_lst:"<<endl;
            // for(auto it=velocity_lst.begin(); it!=velocity_lst.end(); it++)
            // {
            //     cout<<*it<<", ";
            // }
            // cout<<endl;
            // cout<<"aceel_lst:"<<endl;
            // for(auto its=accel_lst.begin(); its!=accel_lst.end(); its++)
            // {
            //     cout<<*its<<", ";
            // }
            // cout<<endl;
            // int vel_length=velocity_lst.size();
            // int accel_length=accel_lst.size();
            // for(int i=0;i<vel_length;i++)
            // {
            //     for(int j=0;j<accel_length;j++)
            //     {
            //         cout<<thr_and_brake_lst[i][j]<<",";
            //     }
            //     cout<<endl;
            // }
            // cout<<"end---end---end---"<<endl;
            // cout<<"lookup result="<<table_value<<endl;
            // cout<<"***********************"<<endl;
            if(table_value>=0.0)
            {
                if(table_value>=1.0) table_value=1.0;
                msg.throttle=table_value;
                msg.brake=0.0;
            }
            else
            {
                if(table_value<=-1.0) table_value=-1.0;
                msg.throttle=0.0;
                msg.brake=table_value;
            }
            msg.steer=0.0;
            controller_pub.publish(msg);
            counter++;
            loop_rate.sleep();
        }
        
    }

    void Controller::readWaypoints()
    {
        ENUCoord point;
        ifstream filew("/home/saxijing/carla-ros-bridge/catkin_ws/data/reference_point/Town07flat_waypoints_and_roadEdge_map.csv");
        if(!filew.is_open())
        {
            cerr<<"无法打开文件"<<endl;
            return;
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
                        case 7:
                            point.x=stod(itemw);
                            break;
                        case 8:
                            point.y=stod(itemw);
                            break;
                        case 9:
                            point.velocity_angle=stod(itemw);
                            break;
                        case 11:
                            point.curvature=stod(itemw);
                            break;
                        // case 25:
                        //     s_w.push_back(stod(itemw));
                            //break;
                    }
                }
                catch(const invalid_argument &e)
                {    cerr<<"转换错误："<<e.what()<<endl;}
                catch(const out_of_range &e)
                {    cerr<<"值超出范围："<<e.what()<<endl;}

            }
            point.velocity_x=0.0;
            point.velocity_y=0.0;
            point.accel_x=0;
            point.accel_y=0.0;
            point.accel=0.0;
            track_waypoints.push_back(point);
            row++;
        }
        filew.close();
    }

    void Controller::readThrottleAndBrakeCalibTable()
    {
        //ROS_INFO("Start to read throtthle and brake calibration table!");
        velocity_lst.clear();
        accel_lst.clear();
        thr_and_brake_lst.resize(1001);
        ifstream filew("/home/saxijing/carla-ros-bridge/catkin_ws/src/ros-bridge/zeus_controller/data/throttle_brake_tabler.csv");
        if(!filew.is_open())
        {
            cerr<<"无法打开文件"<<endl;
            return;
        }
        string linew;
        //getline(filew,linew);
        int row=0;
        while(getline(filew, linew))
        {
            stringstream sss(linew);// 用于将行内容分割为单独的字段
            string itemw;
            //cout<<"row="<<row<<endl;
            for(int col=0; getline(sss, itemw, ','); ++col)
            {
                try
                {
                    // first row fulfill velocity_lst;
                    if(row==0)
                        accel_lst.push_back(stod(itemw));
                    else
                    {
                        if(col==0) //fulfill accel_lst
                            velocity_lst.push_back(stod(itemw));
                        else //fulfill thr_and_brake_lst
                            thr_and_brake_lst[row-1].push_back(stod(itemw));
                    }
                }
                catch(const invalid_argument &e)
                {    cerr<<"转换错误："<<e.what()<<endl;}
                catch(const out_of_range &e)
                {    cerr<<"值超出范围："<<e.what()<<endl;}
            }
            row++;
        }
        filew.close();
        //由于二维表头，第一个行第一列元素为空白
        accel_lst.erase(accel_lst.begin());
        //ROS_INFO("Read throtthle and brake calibration table Succeed");
    }

    double Controller::lookupThrottleAndBrakeCalibTable(const double &vel, const double &accel)
    {
        //ROS_INFO("look up table value!");
        int target_row=(int)(vel*100)/5;
        int target_col=(int)((accel-accel_lst[0])*100)/5;
        int row_size=velocity_lst.size();
        int col_size=accel_lst.size();
        double row_percent=0.0;
        double col_percent=0.0;
        double lookup_result;
        if((target_row>=row_size-1) &&(target_col>=col_size-1))
            return thr_and_brake_lst[row_size-1][col_size-1];
        
        // calculate increment percentage
        if(target_row>=row_size-1) row_percent=0.0;
        else row_percent=(vel-velocity_lst[target_row])/(velocity_lst[target_row+1]-velocity_lst[target_row]);

        if(target_col>=col_size-1) col_percent=0.0;
        else col_percent=(accel-accel_lst[target_col])/(accel_lst[target_col+1]-accel_lst[target_col]);

        if (target_row>=row_size-1)
            lookup_result=thr_and_brake_lst[target_row][target_col]+col_percent*(thr_and_brake_lst[target_row][target_col+1]-thr_and_brake_lst[target_row][target_col]);
        else if(target_col>=col_size-1)
            lookup_result=thr_and_brake_lst[target_row][target_col]+row_percent*(thr_and_brake_lst[target_row+1][target_col]-thr_and_brake_lst[target_row][target_col]);
        else
            lookup_result=thr_and_brake_lst[target_row][target_col]+row_percent*(thr_and_brake_lst[target_row+1][target_col]-thr_and_brake_lst[target_row][target_col])+col_percent*(thr_and_brake_lst[target_row][target_col+1]-thr_and_brake_lst[target_row][target_col]);

        return lookup_result;
    }
}