#pragma once

#include "ControllerCommon.h"

class PitchController: public LinearController {
public:
    PitchController(AP_AHRS &ahrs);
    void update(float theta_cmd_deg, const Accel &accel_max);
    void reset();

    // Output
    float ry_command;
    log_PitchCtrl log;

private:
    LowPassFilter<float> ax_filter;
    float theta_err_i; // Integrator state
    int ry_saturation; // Ry saturation state [-1, 0, 1]
};

class RollYawController: public LinearController {
public:
    RollYawController(AP_AHRS &ahrs);
    void update(float phi_cmd_deg, const Accel &accel_max);

    // Output
    float rx_command;
    float rz_command;
    log_RollYawCtrl log;

private:
    float r_deg_prev; // Previous yaw rate
    float r_deg_hp; // Hi-passed yaw rate
    float phi_err_i; // Integrator state
    int rx_saturation; // Rx saturation state [-1, 0, 1]
    int rz_saturation; // Rz saturation state [-1, 0, 1]
};
