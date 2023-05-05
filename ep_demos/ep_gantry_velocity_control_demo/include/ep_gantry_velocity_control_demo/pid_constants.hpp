#ifndef EP_GANTRY_VELOCITY_CONTROL_DEMO__PID_CONSTANTS_HPP
#define EP_GANTRY_VELOCITY_CONTROL_DEMO__PID_CONSTANTS_HPP

namespace ep_gantry_velocity_control_demo {

struct PidConstants
{
public:
    double kp;
    double ki;
    double kd;

    PidConstants() :
        PidConstants(0.0, 0.0, 0.0)
    {
    }
    PidConstants(const double p, const double i, const double d) :
        kp(p),
        ki(i),
        kd(d)
    {
    }
};

}   // namespaces

#endif  // #ifndef EP_GANTRY_VELOCITY_CONTROL_DEMO__PID_CONSTANTS_HPP
