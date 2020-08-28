#pragma once

#include <vector>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Common/AP_Common.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Math/AP_Math.h>

class LinearController {
public:
    template<size_t m, size_t n>
    LinearController(
        AP_AHRS &ahrs,
        const char *stateNames[],
        const char *expectedNames[],
        const double (&K)[m][n]);

    /* Do not allow copies */
    LinearController(const LinearController &other) = delete;
    LinearController &operator=(const LinearController&) = delete;

protected:
	AP_AHRS &ahrs;
    uint64_t t_prev;
    float dt;
    std::vector<const std::vector<float>> K;

    void update_dt();
    void init_gains();
    std::vector<float> matmul(std::vector<float> vec);
};

class PitchController: public LinearController {
public:
    PitchController(AP_AHRS &ahrs);
    void update(float theta_cmd_deg, float speed_scaler);

private:
    float theta_err_i; // Integrator state
    int elev_saturation; // Elevator saturation state [-1, 0, 1]
};

class RollYawController: public LinearController {
public:
    RollYawController(AP_AHRS &ahrs);
    void update(float phi_cmd_deg, float speed_scaler);

private:
    float r_deg_prev; // Previous yaw rate
    float r_deg_hp; // Hi-passed yaw rate
    float phi_err_i; // Integrator state
    int ail_saturation; // Aileron saturation state [-1, 0, 1]
    int rud_saturation; // Aileron saturation state [-1, 0, 1]
};

class ZenithController {
public:
    ZenithController(AP_AHRS &ahrs)
        : ahrs(ahrs) {};

    /* Do not allow copies */
    ZenithController(const ZenithController &other) = delete;
    ZenithController &operator=(const ZenithController&) = delete;

    void stabilize(float theta_cmd_deg, float roll_cmd_deg);

protected:
	AP_AHRS &ahrs;
    PitchController pitch_controller{ahrs};
    RollYawController roll_yaw_controller{ahrs};

    // TEMP
    uint64_t t0 = 0;
};

