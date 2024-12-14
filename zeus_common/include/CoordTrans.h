#ifndef COORDTRANS_H_
#define COORDTRANS_H_
#include<iostream>
#include<cmath>
#include<vector>
#include<fstream>
#include<sstream>
#include<string>
#include<limits>
#include "CommonCoord.h"
using namespace std;
namespace CoordLib
{
    void radian_unify(double& radius);
    void degree_unify(double& degree);
    void find_c2f_project_point_and_control_error(const ENUCoord &pos_host, ENUCoord &project_point, const vector<ENUCoord>& waypoints, const vector<double>& s_value, int &last_index, double &s, double &dkr, vector<double>& error);
    /*
            input--
            ----pos_host: ego vehicle's pos;
            ----project_point: project point in reference line;
            ----vector x, y, theta, curv: x, y, heading angle, curvature lists of reference line
            ----s_value: s value of project point, equal to pos_host's s value in frenet coordinate
            ----last_index: match point's index in last search, and this is the start of next search, for saving calculation
            ----error=[ed, ed_dot, efai, efai_dot, es, kr]
    */

    void find_f2c_project_point(const FrenetCoord &posf_host, ENUCoord &project_point, vector<ENUCoord>& waypoints, const vector<double>& s_value, int &last_index, double &dkr_);

    void ENU2Frenet(const ENUCoord &project_point, const ENUCoord &pos_host, FrenetCoord &pos_frenet, const double &s);
    void Frenet2ENU(const ENUCoord &project_point, const FrenetCoord &pos_frenet, ENUCoord &pos_cart, const double &dkr);

    void ENU2VehCoord(const ENUCoord &pos_enu, const ENUCoord &pos_host, ENUCoord &pos_veh);
    /*
            ----input:
            ----pos_enu: the pos of point to be transfered, in enu coordinate
            ----pos_host: pos of ego vehicle, and it is the origin of new coordinate
            ----pos_veh: pos in vehicle coordinate, transfer result
    */

    void Veh2ENUCoord(const ENUCoord &pos_veh, const ENUCoord &pos_host, ENUCoord &pos_enu);
    /*
            ----input:
            ----pos_veh: the pos of point to be transfered, in vehicle coordinate
            ----pos_host: pos of ego vehicle
            ----pos_enu: pos in enu coordinate, transfer result
    */

    void test();
    /* test function*/
}
#endif