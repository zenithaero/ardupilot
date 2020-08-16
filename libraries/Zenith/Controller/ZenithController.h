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

    void pitch_ctrl(float theta_cmd_deg);
    void roll_yaw_ctrl(float phi_cmd_deg);

	// int32_t get_rate_out(float desired_rate, float scaler);
	// int32_t get_servo_out(int32_t angle_err, float scaler, bool disable_integrator);

private:
    // Common
	AP_AHRS &ahrs;
    uint32_t t_prev;

    // Pitch control
    const std::vector<double> Kp;
    float theta_err_i;

    float clamp(float value, float min, float max);
    void matmul(float** mat, float *vec, size_t m, size_t n, float* res);
};
