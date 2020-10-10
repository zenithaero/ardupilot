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
    float t = (AP_HAL::micros64() - start_time_us) / 1e6f;
    t = CLAMP(t, 0, N * T);
    float v = cosf(2 * M_PI * t / T);

    printf("v: %.2f\n", v);

    SRV_Channels::set_output_scaled(
        SRV_Channel::k_aileron, 
        (int16_t)(v * ControllerData::rollYaw.maxAilDeg * 100.f)
    );
    SRV_Channels::set_output_scaled(
    SRV_Channel::k_elevator, 
        (int16_t)(v * ControllerData::pitch.maxElevDeg * 100.f * 1.f)
    );
    SRV_Channels::set_output_scaled(
        SRV_Channel::k_rudder, 
        (int16_t)(v * ControllerData::rollYaw.maxRudDeg * 100.f * 0.f) // v-tail
    );

}
