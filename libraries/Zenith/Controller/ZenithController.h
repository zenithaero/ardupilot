#pragma once

#include <vector>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Common/AP_Common.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Math/AP_Math.h>
#include <Filter/LowPassFilter.h>
#include <Zenith/Utils/Interp.h>

class LinearController {
public:
    template<size_t n, size_t n_tas, size_t n_k>
    LinearController(
        AP_AHRS &ahrs,
        const char* (&stateNames)[n],
        const char* (&expectedNames)[n],
        const float (&_K_tas)[n_tas],
        const float (&_K_grid)[n_k]);

    ~LinearController();

    /* Do not allow copies */
    LinearController(const LinearController &other) = delete;
    LinearController &operator=(const LinearController&) = delete;

protected:
	AP_AHRS &ahrs;
    uint64_t t_prev;
    float dt;
    // Gridded gain
    LowPassFilter<float> tas_filter;
    std::vector<float> K_grid;
    Interp<float> *K_interp;
    std::vector<std::vector<float>> K;

    void update();
    void init_gains();
};

class PitchController: public LinearController {
public:
    PitchController(AP_AHRS &ahrs);
    void update(float theta_cmd_deg, float speed_scaler);
    void reset();

    // Output
    float elev_command;

private:
    LowPassFilter<float> ax_filter;
    float theta_err_i; // Integrator state
    int elev_saturation; // Elevator saturation state [-1, 0, 1]
};

class RollYawController: public LinearController {
public:
    RollYawController(AP_AHRS &ahrs);
    void update(float phi_cmd_deg, float rudder_deg, float speed_scaler);

    // Output
    float ail_command;
    float rud_command;

private:
    float r_deg_prev; // Previous yaw rate
    float r_deg_hp; // Hi-passed yaw rate
    float phi_err_i; // Integrator state
    int ail_saturation; // Aileron saturation state [-1, 0, 1]
    int rud_saturation; // Rudder saturation state [-1, 0, 1]
};


class SpdAltController: public LinearController {
public:
    SpdAltController(AP_AHRS &ahrs);
    void update(float tas_cmd, float h_cmd);

    // Output
    float pitch_command;
    float thr_command;

private:
    float h_err_i; // Integrator state
    float tas_err_i; // Integrator state
    int thr_saturation; // Throttle saturation state [-1, 0, 1]
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

class DoubletHandler {
public:
    DoubletHandler() {};

    DoubletHandler(SRV_Channel::Aux_servo_function_t _channel, float _deflection, float _duration, float _settle_time)
        : channel(_channel), deflection(_deflection), duration(_duration), settle_time(_settle_time) {};

    void start();
    void clear();
    bool udpate(float last_command);
    
protected:
    // Parameters
    SRV_Channel::Aux_servo_function_t channel;
    float deflection;
    float duration;
    float settle_time;

    // Vars
    uint64_t t_start = 0;
    LowPassFilter<float> command_filter;
};

class ZenithController {
public:
    ZenithController(AP_AHRS &_ahrs);

    /* Do not allow copies */
    ZenithController(const ZenithController &other) = delete;
    ZenithController &operator=(const ZenithController&) = delete;

    void stabilize(float theta_cmd_deg, float roll_cmd_deg, float rudder_deg);
    void update_spd_alt(float tas_cmd, float h_cmd);
    float get_throttle_command(); 
    float get_pitch_command(); 
    float get_roll_command();
    float get_elev_command();
    float get_ail_command();
    float get_rudder_command();

    // Doublets
    DoubletHandler ail_doublet;
    DoubletHandler elev_doublet;
    DoubletHandler rud_doublet;

protected:
	AP_AHRS &ahrs;
    PitchController pitch_controller{ahrs};
    RollYawController roll_yaw_controller{ahrs};
    SpdAltController spd_alt_controller{ahrs};
    // CrossTrackController xtrack_controller{ahrs};

    // TEMP
    uint64_t t0 = 0;
};

