#pragma once

#include "ControllerCommon.h"

class SpdAltController: public LinearController {
public:
    SpdAltController(AP_AHRS &ahrs);
    void update(float tas_cmd, float h_cmd, const Accel &accel_max);
    void reset();

    // Output
    float pitch_command;
    float ax_command;
    log_SpdAltCtrl log;

private:
    float h_err_i; // Integrator state
    float tas_err_i; // Integrator state
    int ax_saturation; // Ax saturation state [-1, 0, 1]
    int pitch_saturation; // Pitch saturation state [-1, 0, 1]
};

// class CrossTrackController {
// public:
//     CrossTrackController(AP_AHRS &ahrs) : ahrs(ahrs) {};
//     void update_waypoint(const Location &prev_WP, const Location &next_WP, float dist_min);
//     void update_loiter(const Location &center_WP, float radius, int8_t loiter_direction);

// private:
//     AP_AHRS &ahrs; 
// };
