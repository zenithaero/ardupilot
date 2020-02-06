/**
 * @brief Z1 Simulator class
 * @date Created Jan 17, 2020
 * @author Bertrand Bevillard <bertrand@zenithaero.com>
 */

#pragma once

#include "SIM_Aircraft.h"
#include <Filter/LowPassFilter.h>

namespace SITL
{
/*
  a very simple plane simulator
 */
class Z1 : public Aircraft
{
public:
    Z1(const char *frame_str);

    /* update model by one time step */
    virtual void update(const struct sitl_input &input) override;

    /* static object creator */
    static Aircraft *create(const char *frame_str)
    {
        return new Z1(frame_str);
    }

protected:
    const float hover_throttle = 0.7f;
    const float air_density = 1.225; // kg/m^3 at sea level, ISA conditions
    float alpha;
    float beta;
    // TEMP
    float circleAngle;

    struct
    {
        // from last_letter skywalker_2013/aerodynamics.yaml
        // thanks to Georacer!
        float s = 0.45;
        float b = 1.88;
        float c = 0.24;
        float c_lift_0 = 0.56;
        float c_lift_deltae = 0;
        float c_lift_a = 6.9;
        float c_lift_q = 0;
        float mcoeff = 50;
        float oswald = 0.9;
        float alpha_stall = 0.4712;
        float c_drag_q = 0;
        float c_drag_deltae = 0.0;
        float c_drag_p = 0.1;
        float c_y_0 = 0;
        float c_y_b = -0.98;
        float c_y_p = 0;
        float c_y_r = 0;
        float c_y_deltaa = 0;
        float c_y_deltar = -0.2;
        float c_l_0 = 0;
        float c_l_p = -1.0;
        float c_l_b = -0.12;
        float c_l_r = 0.14;
        float c_l_deltaa = 0.25;
        float c_l_deltar = -0.037;
        float c_m_0 = 0.045;
        float c_m_a = -0.7;
        float c_m_q = -20;
        float c_m_deltae = 1.0;
        float c_n_0 = 0;
        float c_n_b = 0.25;
        float c_n_p = 0.022;
        float c_n_r = -1;
        float c_n_deltaa = 0.00;
        float c_n_deltar = 0.1;
        float deltaa_max = 0.3491;
        float deltae_max = 0.3491;
        float deltar_max = 0.3491;
        // the X CoG offset should be -0.02, but that makes the plane too tail heavy
        // in manual flight. Adjusted to -0.15 gives reasonable flight
        Vector3f CGOffset{-0.15, 0, -0.05};
    } coefficient;

    float thrust_scale;
    // Launcher
    bool have_launcher;
    float launch_speed;
    float launch_time;
    uint64_t launch_start_ms = 0;

    float liftCoeff() const;
    float dragCoeff() const;
    Vector3f getForce(float inputAileron, float inputElevator, float inputRudder) const;
    Vector3f getTorque(float inputAileron, float inputElevator, float inputRudder, float inputThrust, const Vector3f &force) const;
    void calculate_forces(const struct sitl_input &input, Vector3f &rot_accel, Vector3f &body_accel);
};
} // namespace SITL


// RCMAP_PITCH	     3
// RCMAP_ROLL	     2
// RCMAP_THROTTLE	 1
// RCMAP_YAW	     4