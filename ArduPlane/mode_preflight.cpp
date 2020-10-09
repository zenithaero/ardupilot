#include "mode.h"
#include "Plane.h"
#include <Zenith/constants.h>

bool ModePreflight::_enter()
{
    start_time_us = AP_HAL::micros64();
    return true;
}

void ModePreflight::update()
{
    // Parameters
    float T = 2; // Swing period
    float N = 3; // Number of repetitions

    // Compute v
    float t = (float)(AP_HAL::micros64() - start_time_us);
    t = CLAMP(t, 0, N * T);
    float v = cosf(2 * M_PI * t / T);

    printf("preflight update: %.2f\n", v);

    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, v * ControllerData::rollYaw.maxAilDeg);
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, v * ControllerData::pitch.maxElevDeg);
    SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, v * ControllerData::rollYaw.maxRudDeg);
}

