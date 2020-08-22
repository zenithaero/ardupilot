/**
 * @brief Zenith controller
 * @date Created August 08, 2020
 * @author Bertrand Bevillard <bertrand@zenithaero.com>
 */

#include <AP_HAL/AP_HAL.h>
#include "ZenithController.h"
#include <assert.h>
#include <Zenith/constants.h>

extern const AP_HAL::HAL& hal;

void ZenithController::stabilize() {
	// Full stabilisation, or input (/state) based mode?
	// Ignore ardupilot state machine?
}


template<size_t m, size_t n>
LinearController::LinearController(
	AP_AHRS &ahrs,
	const char *stateNames[],
	const char *expectedNames[],
	const double (&K)[m][n]) : ahrs(ahrs) {
	for (size_t j = 0; j < n; j++) {
		if (strcmp(stateNames[j], expectedNames[j]) != 0) {
			fprintf(stderr, "State mismatch: %s (expected %s)\n", stateNames[j], expectedNames[j]);
			exit(1);
		}
	}
	for (size_t i = 0; i < m; i++) {
		const std::vector<float> row(K[i], K[i] + n);
		this->K.push_back(row);
	}
}

float LinearController::clamp(float value, float min, float max) {
	return MAX(min, MIN(max, value));
}

void LinearController::update_dt() {
	uint32_t t_now = AP_HAL::millis();
	dt = (float)(t_now - t_prev) / 1e3f;
	if (t_prev == 0 || dt > 1)
		dt = 0.f;
	t_prev = t_now;
}

PitchController::PitchController(AP_AHRS &ahrs)
	: LinearController(
		ahrs,
		ControllerData::pitch.stateNames,
		(const char*[]) {"thetaErrDeg", "thetaErrInt", "<qDeg>", "<hDot>"},
		ControllerData::pitch.K
	) {};

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

void PitchController::update(float theta_cmd_deg) {
	update_dt();
	float theta_deg = (float)ahrs.pitch_sensor / 100.f;
	float theta_err_deg = theta_cmd_deg - theta_deg;
	theta_err_deg = clamp(theta_err_deg, -ControllerData::pitch.maxErrDeg, ControllerData::pitch.maxErrDeg);

	// Determine delta time
	float di = theta_err_deg * dt;
	// Anti-windup - Make sure the index is appropriate
	if (sgn(di * K[0][1]) != sgn(elev_saturation))
		theta_err_i += di;

	// Clip the integrator
	float max_i = INFINITY; // TODO: export max_i with the gains ?
	theta_err_i = clamp(theta_err_i, -max_i, max_i);

	// Retreive q & hDot
	float qDeg = ToDeg(ahrs.get_gyro().y);
	Vector3f vel;
	if (!ahrs.get_velocity_NED(vel))
		vel.z = 0;
	float hDot = -vel.z;

	// Prepare the error vector
	std::vector<float> vec = {
		theta_err_deg,
		theta_err_i,
		qDeg,
		hDot
	};

	// Compute the controller output
	std::vector<float> res = matmul(vec);
	if (res.size() != 1)
		printf("Invalid result size\n");
	float elev = res[0];

	// Add feedforward
	elev += ControllerData::pitch.FF;

	// Clip the elevator
	float elev_clipped = clamp(elev, -ControllerData::pitch.maxElevDeg, ControllerData::pitch.maxElevDeg);
	elev_saturation = sgn(elev - elev_clipped);

	// Assign the output
	int16_t elev_cd = (int16_t)(elev_clipped * 100);
	SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, elev_cd);
}
