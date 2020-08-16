/**
 * @brief Zenith controller
 * @date Created August 08, 2020
 * @author Bertrand Bevillard <bertrand@zenithaero.com>
 */

#include <AP_HAL/AP_HAL.h>
#include "ZenithController.h"
#include <Zenith/Controller/ZenithControllerData.h>
#include <assert.h>

extern const AP_HAL::HAL& hal;

float ZenithController::clamp(float value, float min, float max) {
	return MAX(min, MIN(max, value));
}


void ZenithController::pitch_ctrl(float theta_cmd_deg) {
	float theta_deg = (float)ahrs.pitch_sensor / 100.f;
	float theta_err_deg = theta_cmd_deg - theta_deg;
	theta_err_deg = clamp(theta_err_deg, -ZenithControllerData::pitch.maxErrDeg, ZenithControllerData::pitch.maxErrDeg);
	// Determine delta time
	uint32_t t_now = AP_HAL::millis();
	float dt = (float)(t_now - t_prev) / 1e3f;
	if (t_prev == 0 || dt > 1)
		dt = 0.f;
	t_prev = t_now;
	theta_err_i += theta_err_deg * dt;
	// Clip the integrator
	float max_i = INFINITY; // TODO: export max_i with the gains ?
	theta_err_i = clamp(theta_err_i, -max_i, max_i);
}
