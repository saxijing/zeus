#include "PIDcontroller.h"

using namespace std;
using namespace CoordLib;

namespace controller
{
    PIDcontroller::PIDcontroller()
    {
        //ROS_INFO("PID controller constructed!");
    }

    PIDcontroller::~PIDcontroller()
    {
        //ROS_INFO("PID Controller destroyed!");
    }
    
    void PIDcontroller::initialize(const double &kp_, const double &ki_, const double &kd_, const double &dt_)
    {
        //ROS_INFO("PID controller coeffient initialized!");
        kp=kp_;
        ki=ki_;
        kd=kd_;
        dt=dt_;
        previous_error=0.0;
        previous_output=0.0;
        integral=0.0;
        differential=0.0;
        error_sum=0.0;
        error_diff=0.0;
        first_hit=true;
    }

    double PIDcontroller::pid_control(const double &err)
    {
        error=err;
        double current_output=0.0;
        if(fabs(integral)>5.0)
            Reset();
    
        error_sum+=error;
        error_diff=error-previous_error;
        integral+=error_sum*dt;
        /*if current is the first frame after reset, differential=last differential (it can be equal to 0.0),  else differential=error_diff/dt */
        if(first_hit)
            first_hit=false;
        else
            differential=error_diff/dt;
        
        current_output=kp*error+ki*integral+kd*differential;
        previous_output=current_output;
        return current_output;
    }

    void PIDcontroller::Reset()
    {
        integral=0.0;
        differential=0.0;
        previous_error=0.0;
        first_hit=true;
    }
}