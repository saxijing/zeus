#ifndef COMMONHEADER_H_
#define COMMONHEADER_H_

struct ENUCoord
{
    //time
    unsigned int year;
    unsigned char month;
    unsigned char day;
    unsigned char hour;
    unsigned char min;
    unsigned char sec;
    double msec;

    double sim_time;

    //position,m
    double x;
    double y;
    double z;

    //velocity, m/s
    double velocity;
    double velocity_x;
    double velocity_y;
    double velocity_z;

    //acceleratio, m/s2
    double accel;
    double accel_x;
    double accel_y;
    double accel_z;

    //angle, radius
    double yaw;
    double pitch;
    double roll;
    double velocity_angle;

    //angle velocity,
    double yaw_rate;
    double pitch_rate;
    double roll_rate;

    //curvature,1/m
    double curvature;
};

struct FrenetCoord
{
    //time
    unsigned int year;
    unsigned char month;
    unsigned char day;
    unsigned char hour;
    unsigned char min;
    unsigned char sec;
    double msec;

    double sim_time;

    double s;
    double ds;
    double dds;

    double l;
    double dl;
    double ddl;

    //dl/ds
    double dl_;
    double ddl__;
};

struct VehicleCoord
{
    //time
    unsigned int year;
    unsigned char month;
    unsigned char day;
    unsigned char hour;
    unsigned char min;
    unsigned char sec;
    double msec;

    double sim_time;
     //position
    double veh_x;
    double veh_y;
    double veh_z;

    //velocity
    double veh_v;
    double veh_vx;
    double veh_vy;
    double veh_vz;

    //acceleration
    double veh_accel;
    double veh_ax;
    double veh_ay;
    double veh_az;

    //angle
    double veh_yaw;
    double veh_pitch;
    double veh_roll;
    
    //curvature
    double veh_curv;
};

struct vehicleParam
{
    double vehicle_mass;
    double center2frontaxies;
    double center2rearaxies;
    double inertial_moment;
    double front_wheel_lat_stiff;
    double rear_wheel_lat_stiff;
    double front_wheel_max_angle; //degree
    double rear_wheel_max_angle;
};
#endif