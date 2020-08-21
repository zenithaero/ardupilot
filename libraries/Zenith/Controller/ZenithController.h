#pragma once

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Common/AP_Common.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Math/AP_Math.h>
#include <vector>

class ZenithController {
public:
    ZenithController(AP_AHRS &ahrs)
        : ahrs(ahrs) {};

    /* Do not allow copies */
    ZenithController(const ZenithController &other) = delete;
    ZenithController &operator=(const ZenithController&) = delete;

protected:
	AP_AHRS &ahrs;
};


class LinearController {
public:
    typedef struct {
        const size_t m;
        const size_t n;
    } dim_t;

    LinearController(AP_AHRS &ahrs, dim_t dim, char *stateNames[], char *expectedNames[], float *K[]);

    /* Do not allow copies */
    LinearController(const LinearController &other) = delete;
    LinearController &operator=(const LinearController&) = delete;

protected:
	AP_AHRS &ahrs;
    uint32_t t_prev;
    float dt;
    std::vector<const std::vector<float>> K;
    dim_t dim;

    float clamp(float value, float min, float max);
    void update_dt();
    void init_gains();
    std::vector<float> matmul(std::vector<float> vec);
};

class PitchController: public LinearController {
public:
    PitchController(AP_AHRS &ahrs);
    void update(float theta_cmd_deg);

private:
    float theta_err_i; // Integrator state
    int elev_saturation; // Elevator saturation state [-1, 0, 1]
};
