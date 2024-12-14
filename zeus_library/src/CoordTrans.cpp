#include "CoordTrans.h"
using namespace std;

namespace CoordLib
{
    void radian_unify(double& radius)
    {
        while(radius<=-M_PI ||radius>M_PI)
        {
            while(radius<=-M_PI)
                radius+=2*M_PI;
        
            while(radius>M_PI)
                radius-=2*M_PI;
        }
    }

    void degree_unify(double& degree)
    {
        while(degree<=-180.0 ||degree>180.0)
        {
            while(degree<=-180.0)
                degree+=360.0;
            while(degree>180.0)
                degree-=360.0;
        }
    }

    void find_c2f_project_point_and_control_error(const ENUCoord &pos_host, ENUCoord &project_point, const vector<ENUCoord>& waypoints, const vector<double>& s_value, int &last_index, double &s, double &dkr, vector<double>& error)
    {
        /*
        input--
        ----pos_host: ego vehicle's pos;
        ----project_point: project point in reference line;
        ----vector x, y, theta, curv: x, y, heading angle, curvature lists of reference line
        ----s_value: s value of project point, equal to pos_host's s value in frenet coordinate
        ----last_index: match point's index in last search, and this is the start of next search, for saving calculation
        */

        //error=[ed, ed_dot, efai, efai_dot, es, kr]
        if(last_index<=0)
            last_index=0;
        if(waypoints.size()!=s_value.size())
        {
            cout<<"find_c2f_project point reference waypoints length error!"<<endl;
            return;
        }
        double min_dist=numeric_limits<double>::infinity();
        //find match point index
        for(int i=last_index; i<waypoints.size(); i++)
        {
            if(pow(pos_host.x-waypoints[i].x,2)+pow(pos_host.y-waypoints[i].y,2)<=min_dist)
            {
                min_dist=pow(pos_host.x-waypoints[i].x,2)+pow(pos_host.y-waypoints[i].y,2);
                last_index=i; //last_index
            }
        }
        double theta_m=waypoints[last_index].velocity_angle;
        double km=waypoints[last_index].curvature;
        vector<double>tm={cos(theta_m), sin(theta_m)};
        vector<double>nm={-sin(theta_m), cos(theta_m)};
        vector<double>rh={pos_host.x, pos_host.y};
        vector<double>rm={waypoints[last_index].x, waypoints[last_index].y};
        vector<double>vh={pos_host.velocity_x, pos_host.velocity_y};
        double theta_h=pos_host.velocity_angle;
        double fai_h=pos_host.yaw;
        
        double ed=(rh[0]-rm[0])*nm[0]+(rh[1]-rm[1])*nm[1];
        double es=(rh[0]-rm[0])*tm[0]+(rh[1]-rm[1])*tm[1];
        double theta_r=theta_m+km*es; //theta_r
        radian_unify(theta_r);
        project_point.velocity_angle=theta_r;
        project_point.curvature=km;//kr
        double v_long=vh[0]*cos(theta_h)+vh[1]*sin(theta_h);
        double v_lat=-vh[0]*sin(theta_h)+vh[1]*cos(theta_h);
        double s_dot=(v_long*cos(fai_h-theta_r)-v_lat*sin(fai_h-theta_r))/(1-km*ed);
        double efai=fai_h-theta_r;
        double efai_dot=pos_host.yaw_rate-project_point.curvature*s_dot;
        double ed_dot=v_long*sin(fai_h-theta_r)+v_lat*cos(fai_h-theta_r);

        project_point.x=rm[0]+es*cos(theta_m); //xr
        project_point.y=rm[1]+es*sin(theta_m); //yr
        error={ed, ed_dot, efai, efai_dot, es, km}; //error

        int sign_m=es>0?1:-1;
        s=s_value[last_index]+sign_m*(sqrt(pow(project_point.x-rm[0],2)+pow(project_point.y-rm[1],2)));

        if(last_index==waypoints.size()-1||es<0)
            dkr=(km-waypoints[last_index-1].curvature)/(s_value[last_index]-s_value[last_index-1]);
        else
            dkr=(waypoints[last_index+1].curvature-km)/(s_value[last_index+1]-s_value[last_index]);
    }

    void find_f2c_project_point(const FrenetCoord &posf_host, ENUCoord &project_point, vector<ENUCoord>& waypoints, const vector<double>& s_value, int &last_index, double &dkr_)
    {
        if(last_index<=0)
            last_index=0;
        if(waypoints.size()!=s_value.size())
        {
            cout<<"find_f2c_project reference waypoints length error !"<<endl;
            return;
        }
        //find s_n and s_n+1
        for(int i=last_index; i<waypoints.size()-1;i++)
        {
            if(posf_host.s>=s_value[i] && posf_host.s<s_value[i+1])
            {
                last_index=i;
                break;
            }
        }
        //calculate project point
        double theta_b=(waypoints[last_index].velocity_angle+waypoints[last_index+1].velocity_angle)/2;
        vector<double>tn={cos(theta_b), sin(theta_b)};
        vector<double>rd={(posf_host.s-s_value[last_index])*tn[0],(posf_host.s-s_value[last_index])*tn[1]};
        vector<double>rr={waypoints[last_index].x+rd[0],waypoints[last_index].y+rd[1]};
        double theta_r=waypoints[last_index].velocity_angle+waypoints[last_index].curvature*(posf_host.s-s_value[last_index]);
        radian_unify(theta_r);
        double kr=(waypoints[last_index].curvature+waypoints[last_index+1].curvature)/2;
        // output
        // if(posf_host.s-s_value[last_index]<1e-6)
        //     dkr_=0;
        // else
        dkr_=(waypoints[last_index+1].curvature-waypoints[last_index].curvature)/(s_value[last_index+1]-s_value[last_index]);
        project_point.x=rr[0];
        project_point.y=rr[1];
        project_point.velocity_angle=theta_r;
        project_point.curvature=kr;
    }

    void ENU2Frenet(const ENUCoord &project_point, const ENUCoord &pos_host, FrenetCoord &pos_frenet, const double &s, const double &dkr)
    {
        //s has been calculated in find_c2f_project_point_and_control_error()
        double theta_h=pos_host.velocity_angle;
        double kh=pos_host.curvature;
        vector<double>th={cos(theta_h), sin(theta_h)};
        vector<double>nh={-sin(theta_h), cos(theta_h)};
        vector<double>rh={pos_host.x, pos_host.y};
        vector<double>vh={pos_host.velocity_x, pos_host.velocity_y};
        vector<double>ah={pos_host.accel_x, pos_host.accel_y};

        double theta_r=project_point.velocity_angle;
        double kr=project_point.curvature;
        vector<double>tr={cos(theta_r), sin(theta_r)};
        vector<double>nr={-sin(theta_r), cos(theta_r)};
        vector<double>rr={project_point.x, project_point.y};
        double max_inf=numeric_limits<double>::infinity();

        pos_frenet.year=pos_host.year;
        pos_frenet.month=pos_host.month;
        pos_frenet.day=pos_host.day;
        pos_frenet.hour=pos_host.hour;
        pos_frenet.min=pos_host.min;
        pos_frenet.sec=pos_host.sec;
        pos_frenet.msec=pos_host.msec;
        pos_frenet.sim_time=pos_host.sim_time;

        pos_frenet.s=s; 
        pos_frenet.l=(rh[0]-rr[0])*nr[0]+(rh[1]-rr[1])*nr[1];  // l=(rh-rr) dot nr
        
        if(pos_host.velocity<1e-6)
        {
            pos_frenet.ds=0.0;
            pos_frenet.dl=0.0;
            //pos_frenet.ddl__=0.0;

        }
        else
        {
            pos_frenet.ds=(vh[0]*tr[0]+vh[1]*tr[1])/(1-kr*pos_frenet.l);
            pos_frenet.dl=vh[0]*nr[0]+vh[1]*nr[1]; // dl=v dot nr
        }

        if(cos(theta_h-theta_r)<1e-6 && cos(theta_h-theta_r)>-1e-6)
        {
            pos_frenet.dl_=max_inf;//max_inf;
            pos_frenet.ddl__=max_inf;
        }
        else
        {
            pos_frenet.dl_=(1-kr*pos_frenet.l)*tan(theta_h-theta_r);
            pos_frenet.ddl__=-(dkr*pos_frenet.l+kr*pos_frenet.dl_)*tan(theta_h-theta_r)+pow(1-kr*pos_frenet.l,2)*kh/pow(cos(theta_h-theta_r),3)-(1-kr*pos_frenet.l)*kr/pow(cos(theta_h-theta_r),2);
        }
        
        pos_frenet.dds=((ah[0]*tr[0]+ah[1]*tr[1])+pow(pos_frenet.ds,2)*(2*kr*pos_frenet.dl_+dkr*pos_frenet.l))/(1-kr*pos_frenet.l); //original_calculation
        pos_frenet.ddl=(ah[0]*nr[0]+ah[1]*nr[1])-kr*(1-kr*pos_frenet.l)*pow(pos_frenet.ds,2);
    }

    void Frenet2ENU(const ENUCoord &project_point, const FrenetCoord &pos_frenet, ENUCoord &pos_cart, const double &dkr)
    {
        //dkr has been calculated in find_project_point()
        double theta_r=project_point.velocity_angle;
        double kr=project_point.curvature;
        vector<double>rr={project_point.x, project_point.y};
        vector<double>tr={cos(theta_r), sin(theta_r)};
        vector<double>nr={-sin(theta_r), cos(theta_r)};
        pos_cart.x=rr[0]+pos_frenet.l*nr[0];
        pos_cart.y=rr[1]+pos_frenet.l*nr[1];
        pos_cart.velocity=sqrt(pow(pos_frenet.dl,2)+pow(1-kr*pos_frenet.l,2)*pow(pos_frenet.ds,2));
        double delta_theta;
        if(fabs(1-kr*pos_frenet.l)<1e-6)
            if(fabs(pos_frenet.dl_<1e-6)) delta_theta=0.0;
            else if(pos_frenet.dl_>1e-6) delta_theta=0.5*M_PI;
            else delta_theta=1.5*M_PI;
        else
            delta_theta=atan2(pos_frenet.dl_, 1-kr*pos_frenet.l);
        double theta_h=theta_r+delta_theta;
        radian_unify(theta_h);
        pos_cart.velocity_angle=theta_h;
        if(pos_cart.velocity<1e-6) {pos_cart.velocity_angle=0.0;}
        vector<double>th={cos(theta_h), sin(theta_h)};
        vector<double>nh={-sin(theta_h), cos(theta_h)};
        pos_cart.velocity_x=pos_cart.velocity*th[0];
        pos_cart.velocity_y=pos_cart.velocity*th[1];

        double ah1=pos_frenet.ddl+kr*(1-kr*pos_frenet.l)*pow(pos_frenet.ds,2);
        double ah2=pos_frenet.dds*(1-kr*pos_frenet.l)-pow(pos_frenet.s,2)*(dkr*pos_frenet.l+kr*pos_frenet.dl_)-pow(pos_frenet.ds,2)*kr*pos_frenet.dl_;
        double ah=sqrt(pow(ah1,2)+pow(ah2,2));
        double delta_theta_ah;
        if (fabs(ah2<1e-6))
        {
            if(fabs(ah1)<1e-6) delta_theta_ah=0.0;
            else if(ah1>1e-6) delta_theta_ah=0.5*M_PI;
            else delta_theta_ah=1.5*M_PI;
        }
        else
            delta_theta_ah=atan2(ah1,ah2);

        double theta_ah=theta_r+delta_theta_ah;
        if(ah<1e-6) {theta_ah=0.0;}
        pos_cart.accel=ah;
        pos_cart.accel_x=ah*cos(theta_ah);
        pos_cart.accel_y=ah*sin(theta_ah);
        pos_cart.curvature=kr*cos(theta_h-theta_r)/(1-kr*pos_frenet.l);
        pos_cart.curvature+=pow(cos(theta_h-theta_r),2)*(pos_frenet.ddl__*cos(theta_h-theta_r)+(dkr*pos_frenet.l+kr*pos_frenet.dl_)*sin(theta_h-theta_r))/pow(1-kr*pos_frenet.l,2);
    }

    void ENU2VehCoord(const ENUCoord &pos_enu, const ENUCoord &pos_host, ENUCoord &pos_veh)
    {
        /*
        ----input:
        ----pos_enu: the pos of point to be transfered, in enu coordinate
        ----pos_host: pos of ego vehicle, and it is the origin of new coordinate
        ----pos_veh: pos in vehicle coordinate, transfer result
        */
        vector<double>rh={pos_host.x, pos_host.y};
        vector<double>vh={pos_host.velocity_x, pos_host.velocity_y};
        vector<double>ah={pos_host.accel_x, pos_host.accel_y};
        double theta_h=pos_host.velocity_angle;
        double fai_h=pos_host.yaw;
        double curv_h=pos_host.curvature;

        vector<double>rp={pos_enu.x, pos_enu.y};
        vector<double>vp={pos_enu.velocity_x, pos_enu.velocity_y};
        vector<double>ap={pos_enu.accel_x, pos_enu.accel_y};
        double theta_p=pos_enu.velocity_angle;
        double curv_p=pos_enu.curvature;
        double fai_p=pos_enu.yaw;

        pos_veh.x=(rp[0]-rh[0])*cos(fai_h)+(rp[1]-rh[1])*sin(fai_h);
        pos_veh.y=(rp[1]-rh[1])*cos(fai_h)-(rp[0]-rh[0])*sin(fai_h);
        pos_veh.velocity_angle=theta_p-theta_h;
        pos_veh.yaw=fai_p-fai_h;
        radian_unify(pos_veh.velocity_angle);
        radian_unify(pos_veh.yaw);
        double theta_p_trans=pos_veh.velocity_angle;
        double fai_p_trans=pos_veh.yaw;

        pos_veh.velocity_x=(vp[0]-vh[0])*cos(fai_h)+(vp[1]-vh[1])*sin(fai_h);
        pos_veh.velocity_y=(vp[1]-vh[1])*cos(fai_h)-(vp[0]-vh[0])*sin(fai_h);
        pos_veh.velocity=sqrt(pow(pos_veh.velocity_x,2)+pow(pos_veh.velocity_y,2));

        pos_veh.accel_x=(ap[0]-ah[0])*cos(fai_h)+(ap[1]-ah[1])*sin(fai_h);
        pos_veh.accel_y=(ap[1]-ah[1])*cos(fai_h)-(ap[0]-ah[0])*sin(fai_h);
        pos_veh.accel=sqrt(pow(pos_veh.accel_x,2)+pow(pos_veh.accel_y,2));
        pos_veh.curvature=pos_veh.accel_y>0?(-1*fabs(curv_p)):fabs(curv_p);
    }

    void Veh2ENUCoord(const ENUCoord &pos_veh, const ENUCoord &pos_host, ENUCoord &pos_enu)
    {
        /*
        ----input:
        ----pos_veh: the pos of point to be transfered, in vehicle coordinate
        ----pos_host: pos of ego vehicle
        ----pos_enu: pos in enu coordinate, transfer result
        */
        vector<double>rh={pos_host.x, pos_host.y};
        vector<double>vh={pos_host.velocity_x, pos_host.velocity_y};
        vector<double>ah={pos_host.accel_x, pos_host.accel_y};
        double theta_h=pos_host.velocity_angle;
        double curv_h=pos_host.curvature;
        double fai_h=pos_host.yaw;

        vector<double>rv={pos_veh.x, pos_veh.y};
        vector<double>vv={pos_veh.velocity_x, pos_veh.velocity_y};
        vector<double>av={pos_veh.accel_x, pos_veh.accel_y};
        double theta_v=pos_veh.velocity_angle;
        double curv_v=pos_veh.curvature;
        double fai_v=pos_veh.yaw;

        pos_enu.x=rh[0]+rv[0]*cos(fai_h)-rv[1]*sin(fai_h);
        pos_enu.y=rh[1]+rv[0]*sin(fai_h)+rv[1]*cos(fai_h);
        pos_enu.velocity_angle=theta_h+theta_v;
        radian_unify(pos_enu.yaw);
        pos_enu.yaw=fai_h+fai_v;
        radian_unify(pos_enu.yaw);
        pos_enu.velocity_x=vh[0]+vv[0]*cos(fai_h)-vv[1]*sin(fai_h);
        pos_enu.velocity_y=vh[1]+vv[0]*sin(fai_h)+vv[1]*cos(fai_h);
        pos_enu.velocity=sqrt(pow(pos_enu.velocity_x,2)+pow(pos_enu.velocity_y,2));

        pos_enu.accel_x=ah[0]+av[0]*cos(fai_h)-av[1]*sin(fai_h);
        pos_enu.accel_y=ah[1]+av[0]*sin(fai_h)+av[1]*cos(fai_h);
        pos_enu.accel=sqrt(pow(pos_enu.accel_x,2)+pow(pos_enu.accel_y,2));

        pos_enu.curvature=pos_enu.accel_y>0?(-fabs(curv_v)):fabs(curv_v);
    }

    void left2right(ENUCoord &pos)
    {
        // just consider 2D
        pos.y=-pos.y;
        pos.velocity_y=-pos.velocity_y;
        pos.accel_y=-pos.accel_y;
        pos.curvature=-pos.curvature;
        radian_unify(pos.yaw);
        pos.yaw=2*M_PI-pos.yaw;
        radian_unify(pos.yaw);
    }

    void right2left(ENUCoord &pos)
    {
        // just consider 2D
        pos.y=-pos.y;
        pos.velocity_y=-pos.velocity_y;
        pos.accel_y=-pos.accel_y;
        pos.curvature=-pos.curvature;
        radian_unify(pos.yaw);
        pos.yaw=2*M_PI-pos.yaw;
        radian_unify(pos.yaw);
    }

    // void test()
    // {
    //     // ENUCoord ego1;
    //     // ENUCoord ego2;
    //     // // ego1.x=324.731067257828;
    //     // // ego1.y=2.01179954469066;//index=10
    //     // // ego1.yaw=3.1415;
    //     // // ego1.curvature=0.0;
    //     // // ego1.velocity=10.0;
    //     // // ego1.velocity_x=-10.0;
    //     // // ego1.velocity_y=0.0;
    //     // // ego1.accel=0.5;
    //     // // ego1.accel_x=-0.5;
    //     // // ego1.accel_y=0.0;

    //     // // ego2.x=162.567206148599;//index1638
    //     // // ego2.y=1.88949001498851;
    //     // // ego2.yaw=3.3196855915897;
    //     // // ego2.curvature=0.112162042051549;
    //     // // ego2.velocity=5.08;
    //     // // ego2.velocity_x=-5.0;
    //     // // ego2.velocity_y=-0.9;
    //     // // ego2.accel=0.316;
    //     // // ego2.accel_x=-0.3;
    //     // // ego2.accel_y=-0.1;

    //     // ENUCoord proj1[12];
    //     // ENUCoord proj2[12];
    //     // ENUCoord proj1t[12];
    //     // ENUCoord proj2t[12];

    //     // ENUCoord ego1_veh[12];
    //     // ENUCoord ego2_veh[12];
    //     // ENUCoord ego1_enu_fromV[12];
    //     // ENUCoord ego2_enu_fromV[12];
    //     // FrenetCoord ego1_fre[12];
    //     // FrenetCoord ego2_fre[12];
    //     // ENUCoord ego1_enu_fromF[12];
    //     // ENUCoord ego2_enu_fromF[12];
    //     // ENUCoord ego1_veh_fromE[12];
    //     // ENUCoord ego2_veh_fromE[12];
    //     // ENUCoord ego1_veh_backup[12];
    //     // ENUCoord ego2_veh_backup[12];
    //     ENUCoord ego_enu[12];
    //     ENUCoord project[12];
    //     ENUCoord projt[12];
    //     FrenetCoord ego_fre[12];
    //     ENUCoord ego_enu_fromF[12];

    //     //read vehicle coord values
    //     ifstream file("/home/saxijing/carla-ros-bridge/catkin_ws/data/coord_test.csv");
    //     if(!file.is_open())
    //     {
    //         cerr<<"无法打开文件"<<endl;
    //         return;
    //     }
    //     string line;
    //     getline(file,line);
    //     int rowindex=0;
    //     double max_inf=numeric_limits<double>::infinity();

    //     while(getline(file,line))
    //     {
    //         stringstream ss(line); // 用于将行内容分割为单独的字段
    //         string item;

    //         for(int colindex=0; getline(ss, item, ','); ++colindex)
    //         {
    //             try
    //             {
    //                 switch(colindex)
    //                 {
    //                     case 1:
    //                         ego_enu[rowindex].x=stod(item);
    //                         break;
    //                     case 2:
    //                         ego_enu[rowindex].y=stod(item);
    //                         break;
    //                     case 3:
    //                         ego_enu[rowindex].yaw=stod(item);
    //                         break;
    //                     case 4:
    //                         if(item=="INF")
    //                         {
    //                             ego_enu[rowindex].curvature=max_inf;
    //                         }
    //                         else
    //                         {
    //                             ego_enu[rowindex].curvature=stod(item);
    //                         }
    //                         break;
    //                     case 5:
    //                         ego_enu[rowindex].velocity_x=stod(item);
    //                         break;
    //                     case 6:
    //                         ego_enu[rowindex].velocity_y=stod(item);
    //                         break;
    //                     case 7:
    //                         ego_enu[rowindex].velocity=stod(item);
    //                         break;
    //                     case 8:
    //                         ego_enu[rowindex].accel_x=stod(item);
    //                         break;
    //                     case 9:
    //                         ego_enu[rowindex].accel_y=stod(item);
    //                         break;
    //                     case 10:
    //                         ego_enu[rowindex].accel=stod(item);
    //                         break;
    //                 }
    //             }
    //             catch(const invalid_argument &e)
    //             {    cerr<<"转换错误："<<e.what()<<endl;}
    //             catch(const out_of_range &e)
    //             {    cerr<<"值超出范围："<<e.what()<<endl;}

    //         }
    //         rowindex++;
    //     }
    //     file.close();

    //     //read reference line waypoints
    //     vector<ENUCoord> refline_waypoints;
    //     ENUCoord point;
    //     vector<double>s_w;
    //     ifstream filew("/home/saxijing/carla-ros-bridge/catkin_ws/data/reference_point/waypoints_and_roadEdge_map.csv");
    //     if(!filew.is_open())
    //     {
    //         cerr<<"无法打开文件"<<endl;
    //         return;
    //     }
    //     string linew;
    //     getline(filew,linew);
    //     int row=0;
    //     while(getline(filew,linew))
    //     {
    //         stringstream sss(linew); // 用于将行内容分割为单独的字段
    //         string itemw;

    //         for(int col=0; getline(sss, itemw, ','); ++col)
    //         {
    //             try
    //             {
    //                 switch(col)
    //                 {
    //                     case 13:
    //                         point.x=stod(itemw);
    //                         break;
    //                     case 14:
    //                        point.y=stod(itemw);
    //                         break;
    //                     case 15:
    //                         point.yaw=stod(itemw);
    //                         break;
    //                     case 17:
    //                         point.curvature=stod(itemw);
    //                         break;
    //                     case 25:
    //                         s_w.push_back(stod(itemw));
    //                         break;
    //                 }
    //             }
    //             catch(const invalid_argument &e)
    //             {    cerr<<"转换错误："<<e.what()<<endl;}
    //             catch(const out_of_range &e)
    //             {    cerr<<"值超出范围："<<e.what()<<endl;}

    //         }
    //         refline_waypoints.push_back(point);
    //         row++;
    //     }
    //     filew.close();
    //     cout<<"length of waypoints_lst:"<<refline_waypoints.size()<<endl;
    //     cout<<"length of s_value: "<< s_w.size()<<endl;
    //     int length=refline_waypoints.size();
    //     cout<<refline_waypoints[length-1].x<<", "<<refline_waypoints[length-1].y<<", "<<refline_waypoints[length-1].yaw<<endl;

    //     /*test step: 
    //         1. read vehicle coord values;
    //         2. vehicle 2 enu coord;
    //         3. enu 2 frenet;
    //         4. frenet 2 enu
    //         5. enu 2 vehicle
    //     */
    //     //step 1: read vehicle coord values;
    //     for(int j=0; j<12; j++)
    //     {
            
    //         cout<<"ego_enu["<<j<<"]: x="<<ego_enu[j].x<<", y="<<ego_enu[j].y<<", yaw="<<ego_enu[j].yaw<<", curv="<<ego_enu[j].curvature<<", vx="<<ego_enu[j].velocity_x<<", vy="<<ego_enu[j].velocity_y<<", v="<<ego_enu[j].velocity<<
    //                 ", ax="<<ego_enu[j].accel_x<<", ay="<<ego_enu[j].accel_y<<", a="<<ego_enu[j].accel<<endl;
    //     }
    //     cout<<"*********STEP 1. ENU coord read success !*********"<<endl<<endl;
        
    //     //step 2: vehicle 2 enu coord;
    //     // for(int j=0; j<12;j++)
    //     // {
    //     //     Veh2ENUCoord(ego1_veh[j], ego1, ego1_enu_fromV[j]);
    //     //     Veh2ENUCoord(ego2_veh[j], ego2, ego2_enu_fromV[j]);
    //     //     cout<<"ego1_enu_fromV["<<j<<"]: x="<<ego1_enu_fromV[j].x<<", y="<<ego1_enu_fromV[j].y<<", yaw="<<ego1_enu_fromV[j].yaw<<", curv="<<ego1_enu_fromV[j].curvature<<", vx="<<ego1_enu_fromV[j].velocity_x<<", vy="<<ego1_enu_fromV[j].velocity_y<<", v="<<ego1_enu_fromV[j].velocity<<
    //     //             ", ax="<<ego1_enu_fromV[j].accel_x<<", ay="<<ego1_enu_fromV[j].accel_y<<", a="<<ego1_enu_fromV[j].accel<<endl;
    //     //     cout<<"ego2_enu_fromV["<<j<<"]: x="<<ego2_enu_fromV[j].x<<", y="<<ego2_enu_fromV[j].y<<", yaw="<<ego2_enu_fromV[j].yaw<<", curv="<<ego2_enu_fromV[j].curvature<<", vx="<<ego2_enu_fromV[j].velocity_x<<", vy="<<ego2_enu_fromV[j].velocity_y<<", v="<<ego2_enu_fromV[j].velocity<<
    //     //             ", ax="<<ego2_enu_fromV[j].accel_x<<", ay="<<ego2_enu_fromV[j].accel_y<<", a="<<ego2_enu_fromV[j].accel<<endl;
    //     // }
    //     // cout<<"*********STEP 2. Vehicle 2 ENU transform success !*********"<<endl<<endl;

    //     // for(int j=0;j<12;j++)
    //     // {
    //     //     ENU2VehCoord(ego1_enu_fromV[j], ego1, ego1_veh_backup[j]);
    //     //     ENU2VehCoord(ego2_enu_fromV[j], ego2, ego2_veh_backup[j]);
    //     //     cout<<"ego1_veh_backup["<<j<<"]: x="<<ego1_veh_backup[j].x<<", y="<<ego1_veh_backup[j].y<<", yaw="<<ego1_veh_backup[j].yaw<<", curv="<<ego1_veh_backup[j].curvature<<", vx="<<ego1_veh_backup[j].velocity_x<<", vy="<<ego1_veh_backup[j].velocity_y<<", v="<<ego1_veh_backup[j].velocity<<
    //     //                 ", ax="<<ego1_veh_backup[j].accel_x<<", ay="<<ego1_veh_backup[j].accel_y<<", a="<<ego1_veh_backup[j].accel<<endl;
    //     //         cout<<"ego2_veh_backup["<<j<<"]: x="<<ego2_veh_backup[j].x<<", y="<<ego2_veh_backup[j].y<<", yaw="<<ego2_veh_backup[j].yaw<<", curv="<<ego2_veh_backup[j].curvature<<", vx="<<ego2_veh_backup[j].velocity_x<<", vy="<<ego2_veh_backup[j].velocity_y<<", v="<<ego2_veh_backup[j].velocity<<
    //     //                 ", ax="<<ego2_veh_backup[j].accel_x<<", ay="<<ego2_veh_backup[j].accel_y<<", a="<<ego2_veh_backup[j].accel<<endl;
    //     // }
    //     // cout<<"*********STEP 2.1. ENU 2 Vehicle backup transform success !*********"<<endl; 

    //     //step3: enu 2 frenet;
    //     int last_index1=0, last_index2=0;
    //     vector<double>err1;
    //     vector<double>err2;
    //     double s_p1,s_p2;
    //     double dkr1,dkr2;
        
    //     for(int j=0;j<12;j++)
    //     {
    //         find_c2f_project_point_and_control_error(ego_enu[j], project[j], refline_waypoints, s_w, last_index1, s_p1, dkr1, err1);
    //         ENU2Frenet(project[j], ego_enu[j], ego_fre[j], s_p1, dkr1);
            
    //         // find_c2f_project_point_and_control_error(ego2_enu_fromV[j], proj2[j], x_w, y_w, yaw_w, curv_w, s_w, last_index2, s_p2, dkr2, err2);
    //         // ENU2Frenet(proj2[j], ego2_enu_fromV[j], ego2_fre[j], s_p2,dkr2);
           
    //         cout<<"project_c2f["<<j<<"]: x="<<project[j].x<<", y="<<project[j].y<<", yaw="<<project[j].yaw<<", curv="<<project[j].curvature<<",s="<<s_w[last_index1]<<", index="<<last_index1<<endl;
    //         //cout<<"project2_c2f["<<j<<"]: x="<<proj2[j].x<<", y="<<proj2[j].y<<", yaw="<<proj2[j].yaw<<", curv="<<proj2[j].curvature<<", s="<<s_w[last_index2]<<", index="<<last_index2<<endl;
    //         cout<<"ego_fre["<<j<<"]: s="<<ego_fre[j].s<<", ds="<<ego_fre[j].ds<<", dds="<<ego_fre[j].dds<<", l="<<ego_fre[j].l<<
    //                     ", dl="<<ego_fre[j].dl<<", ddl="<<ego_fre[j].ddl<<", dl_="<<ego_fre[j].dl_<<", ddl_="<<ego_fre[j].ddl__<<endl;
    //         // cout<<"ego2_fre["<<j<<"]: s="<<ego2_fre[j].s<<", ds="<<ego2_fre[j].ds<<", dds="<<ego2_fre[j].dds<<", l="<<ego2_fre[j].l<<
    //         //             ", dl="<<ego2_fre[j].dl<<", ddl="<<ego2_fre[j].ddl<<", dl_="<<ego2_fre[j].dl_<<", ddl_="<<ego2_fre[j].ddl__<<endl;
    //         last_index1=0;
    //     }
    //     cout<<"*********STEP 3. ENU 2 Frenet transform success !*********"<<endl<<endl;

    //     //step 4: frenet 2 enu
    //     int last_index3=0, last_index4=0;
    //     double dkr3, dkr4;
    //     for(int j=0;j<12;j++)
    //     {
    //         find_f2c_project_point(ego_fre[j], projt[j], refline_waypoints, s_w, last_index3, dkr3);
    //         Frenet2ENU(projt[j], ego_fre[j], ego_enu_fromF[j], dkr3);
            
    //         // find_f2c_project_point(ego2_fre[j], proj2t[j], x_w, y_w, yaw_w, curv_w, s_w, last_index4, dkr4);
    //         // Frenet2ENU(proj2t[j], ego2_fre[j], ego2_enu_fromF[j], dkr4);
            
    //         cout<<"project_f2c["<<j<<"]: x="<<projt[j].x<<", y="<<projt[j].y<<", yaw="<<projt[j].yaw<<", curv="<<projt[j].curvature<<", s="<<s_w[last_index3]<<", index="<<last_index3<<endl;
    //         //cout<<"project2_f2c["<<j<<"]: x="<<proj2t[j].x<<", y="<<proj2t[j].y<<", yaw="<<proj2t[j].yaw<<", curv="<<proj2t[j].curvature<<", s="<<s_w[last_index4]<<", index="<<last_index4<<endl;
    //         cout<<"ego_enu_fromF["<<j<<"]: x="<<ego_enu_fromF[j].x<<", y="<<ego_enu_fromF[j].y<<", yaw="<<ego_enu_fromF[j].yaw<<", curv="<<ego_enu_fromF[j].curvature<<", vx="<<ego_enu_fromF[j].velocity_x<<", vy="<<ego_enu_fromF[j].velocity_y<<", v="<<ego_enu_fromF[j].velocity<<
    //                 ", ax="<<ego_enu_fromF[j].accel_x<<", ay="<<ego_enu_fromF[j].accel_y<<", a="<<ego_enu_fromF[j].accel<<endl;
    //         // cout<<"ego2_enu_fromF["<<j<<"]: x="<<ego2_enu_fromF[j].x<<", y="<<ego2_enu_fromF[j].y<<", yaw="<<ego2_enu_fromF[j].yaw<<", curv="<<ego2_enu_fromF[j].curvature<<", vx="<<ego2_enu_fromF[j].velocity_x<<", vy="<<ego2_enu_fromF[j].velocity_y<<", v="<<ego2_enu_fromF[j].velocity<<
    //         //         ", ax="<<ego2_enu_fromF[j].accel_x<<", ay="<<ego2_enu_fromF[j].accel_y<<", a="<<ego2_enu_fromF[j].accel<<endl;
    //         last_index3=0;
    //     }
    //     cout<<"*********STEP 4. Frenet 2 ENU transform success !*********"<<endl<<endl;
    //     // //5. enu 2 vehicle
    //     // for(int j=0;j<12;j++)
    //     // {
    //     //     ENU2VehCoord(ego1_enu_fromF[j], ego1, ego1_veh_fromE[j]);
    //     //     ENU2VehCoord(ego2_enu_fromF[j], ego2, ego2_veh_fromE[j]);
    //     //     cout<<"ego1_veh_fromE["<<j<<"]: x="<<ego1_veh_fromE[j].x<<", y="<<ego1_veh_fromE[j].y<<", yaw="<<ego1_veh_fromE[j].yaw<<", curv="<<ego1_veh_fromE[j].curvature<<", vx="<<ego1_veh_fromE[j].velocity_x<<", vy="<<ego1_veh_fromE[j].velocity_y<<", v="<<ego1_veh_fromE[j].velocity<<
    //     //                 ", ax="<<ego1_veh_fromE[j].accel_x<<", ay="<<ego1_veh_fromE[j].accel_y<<", a="<<ego1_veh_fromE[j].accel<<endl;
    //     //         cout<<"ego2_veh_fromE["<<j<<"]: x="<<ego2_veh_fromE[j].x<<", y="<<ego2_veh_fromE[j].y<<", yaw="<<ego2_veh_fromE[j].yaw<<", curv="<<ego2_veh_fromE[j].curvature<<", vx="<<ego2_veh_fromE[j].velocity_x<<", vy="<<ego2_veh_fromE[j].velocity_y<<", v="<<ego2_veh_fromE[j].velocity<<
    //     //                 ", ax="<<ego2_veh_fromE[j].accel_x<<", ay="<<ego2_veh_fromE[j].accel_y<<", a="<<ego2_veh_fromE[j].accel<<endl;
    //     // }
    //     // cout<<"*********STEP 5. ENU 2 Vehicle transform success !*********"<<endl; 
    // }
}