#ifndef EP_GANTRY_VELOCITY_CONTROL_DEMO__VELOCITY_TRAPEZOIDAL_PROFILE_HPP
#define EP_GANTRY_VELOCITY_CONTROL_DEMO__VELOCITY_TRAPEZOIDAL_PROFILE_HPP

namespace ep_gantry_velocity_control_demo {

struct VelTrapProfile
{
    double t_accelerating;
    double t_constant_velocity;
    double acceleration;

    VelTrapProfile() :
        VelTrapProfile(0.0, 0.0, 0.0)
    {
    }
    VelTrapProfile(const double t_acc, const double t_const_vel, const double acc) :
        t_accelerating(t_acc),
        t_constant_velocity(t_const_vel),
        acceleration(acc)
    {
    }

    /** Calculate the desired velocity at an elapsed time.
     *  @param dt The number of seconds since the start of the motion to
     *  evaluate the velocity at.
     *  @return The desired velocity.
     */
    double evaluate_at(const double dt) const
    {
        // Given the time elapsed, calculate the desired velocity
        double desi_vel = 0.0;
        if (dt > 0.0)
        {
            // The theoretical peak velocity of the motion
            const double peak_vel = acceleration * t_accelerating;

            if (dt < t_accelerating)
            {
                desi_vel = peak_vel * (dt / t_accelerating);
            }
            else if (dt < (t_accelerating + t_constant_velocity))
            {
                desi_vel = peak_vel;
            }
            else if (dt < ((2 * t_accelerating) + t_constant_velocity))
            {
                desi_vel = peak_vel * (1 - ((dt - t_accelerating - t_constant_velocity) / t_accelerating));
            }
            // else desi velocity is already 0.0
        }
        // else desi velocity is already 0.0

        return desi_vel;
    }
};

}   // namespaces

#endif  // #ifndef EP_GANTRY_VELOCITY_CONTROL_DEMO__VELOCITY_TRAPEZOIDAL_PROFILE_HPP
