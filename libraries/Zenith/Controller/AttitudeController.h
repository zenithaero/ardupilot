#pragma once

#include "ControllerCommon.h"

class PitchController: public LinearController {
public:
    PitchController(AP_AHRS &ahrs);
    void update(float theta_cmd_deg);
    void reset();

    // Output
    float elev_command;
    log_PitchCtrl log;

private:
    LowPassFilter<float> ax_filter;
    float theta_err_i; // Integrator state
    int elev_saturation; // Elevator saturation state [-1, 0, 1]
};

class RollYawController: public LinearController {
public:
    RollYawController(AP_AHRS &ahrs);
    void update(float phi_cmd_deg, float rudder_deg);

    // Output
    float ail_command;
    float rud_command;
    log_RollYawCtrl log;

private:
    float r_deg_prev; // Previous yaw rate
    float r_deg_hp; // Hi-passed yaw rate
    float phi_err_i; // Integrator state
    int ail_saturation; // Aileron saturation state [-1, 0, 1]
    int rud_saturation; // Rudder saturation state [-1, 0, 1]
};
