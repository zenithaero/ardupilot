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

void ZenithController::stabilize(float theta_cmd_deg, float phi_cmd_deg) {
	// Full stabilisation, or input (/state) based mode?
	// Ignore ardupilot state machine?

	// TEMP
	if (t0 == 0)
        t0 = AP_HAL::micros64();
    float dt = (AP_HAL::micros64() - t0) / 1e6f;

	// Compute speed scaler
	float speed_scaler = 1;
	float airspeed;
    if (ahrs.airspeed_estimate(airspeed)) {
		airspeed = MAX(1, airspeed);
		speed_scaler = ControllerData::constants.scalingSpeed / airspeed;
		speed_scaler = CLAMP(speed_scaler, 0.2, 2);
    }
	// printf("airspeed %.2f, scaler %.2f\n", airspeed, speed_scaler);

	// phi_cmd_deg = 0;
	// if (dt > 70)
	// 	phi_cmd_deg += 5;

	// Stabilize pitch, roll & yaw
	pitch_controller.update(theta_cmd_deg, speed_scaler);
	roll_yaw_controller.update(phi_cmd_deg, speed_scaler);
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

void LinearController::update_dt() {
	uint64_t t_now = AP_HAL::micros64();
	dt = (float)(t_now - t_prev) / 1e6f;
	if (t_prev == 0 || dt > 1)
		dt = 0.f;
	t_prev = t_now;
}

std::vector<float> LinearController::matmul(std::vector<float> vec) {
	std::vector<float> rtn(K.size(), 0);
	if (vec.size() != K[0].size()) {
		fprintf(stderr, "Invalid vector size: %lu, expected %lu\n", vec.size(), K[0].size());
		return rtn;
	}
	for (size_t i = 0; i < K.size(); i++)
		for (size_t j = 0; j < K[0].size(); j++)
			rtn[i] += K[i][j] * vec[j];
	return rtn;
}

PitchController::PitchController(AP_AHRS &ahrs)
	: LinearController(
		ahrs,
		ControllerData::pitch.stateNames,
		(const char*[]) {"thetaErrDeg", "thetaErrInt", "<qDeg>", "<hDot>"},
		ControllerData::pitch.K
	) {};

void PitchController::update(float theta_cmd_deg, float speed_scaler) {
	update_dt();

	float theta_deg = ahrs.pitch_sensor / 100.f;
	float theta_err_deg = theta_cmd_deg - theta_deg;
	theta_err_deg = CLAMP(theta_err_deg, -ControllerData::pitch.maxErrDeg, ControllerData::pitch.maxErrDeg);

	// Handle integrator
	if (dt > 1)
		theta_err_i = 0;
	float di = theta_err_deg * dt;
	// Anti-windup - Make sure the index is appropriate
	if (sgn(di * K[0][1]) != sgn(elev_saturation) || true)
		theta_err_i += di;

	// Clamp the integrator
	float max_i = INFINITY; // TODO: export max_i with the gains ?
	theta_err_i = CLAMP(theta_err_i, -max_i, max_i);

	// Retreive q & h_dot
	float q_deg = ToDeg(ahrs.get_gyro().y);
	Vector3f vel;
	if (!ahrs.get_velocity_NED(vel))
		vel.z = 0;
	float h_dot = -vel.z;

	// Prepare the error vector
	std::vector<float> vec = {
		theta_err_deg,
		theta_err_i,
		q_deg,
		h_dot
	};

	// Compute the controller output
	std::vector<float> res = matmul(vec);
	if (res.size() != 1)
		fprintf(stderr, "Invalid output size: %lu\n", res.size());
	
	float elev = res[0] * speed_scaler;

	// Add feedforward
	elev += ControllerData::pitch.FF;

	// Clamp the elevator
	float elev_clamped = CLAMP(elev, -ControllerData::pitch.maxElevDeg, ControllerData::pitch.maxElevDeg);
	elev_saturation = sgn(elev - elev_clamped);

	// Assign the output
	printf("theta_cmd_deg %.2f, theta_deg %.2f, elev %.2f\n", theta_cmd_deg, theta_deg, elev_clamped);
	int16_t elev_cd = (int16_t)(elev_clamped * 100);
	SRV_Channels::move_servo(SRV_Channel::k_elevator, -elev_cd, -1500, 1500);
}

RollYawController::RollYawController(AP_AHRS &ahrs)
	: LinearController(
		ahrs,
		ControllerData::rollYaw.stateNames,
		(const char*[]) {"phiErrDeg", "phiErrInt", "<pDeg>", "rDegHP"},
		ControllerData::rollYaw.K
	) {};

void RollYawController::update(float phi_cmd_deg, float speed_scaler) {
	update_dt();
	float phi_deg = ahrs.roll_sensor / 100.f;
	float phi_err_deg = phi_cmd_deg - phi_deg;
	phi_err_deg = CLAMP(phi_err_deg, -ControllerData::rollYaw.maxErrDeg, ControllerData::rollYaw.maxErrDeg);

	// Handle integrator
	if (dt > 1)
		phi_err_i = 0;
	float di = phi_err_deg * dt;
	// Anti-windup - Make sure the index is appropriate
	if (sgn(di * K[0][1]) != sgn(ail_saturation) || true)
		phi_err_i += di;

	// Clamp the integrator
	float max_i = INFINITY; // TODO: export max_i with the gains ?
	phi_err_i = CLAMP(phi_err_i, -max_i, max_i);

	// Retreive q & h_dot
	float p_deg = ToDeg(ahrs.get_gyro().x);
	float r_deg = ToDeg(ahrs.get_gyro().z);

	// UNUSED
	// Apply a high-pass filter to the rate
	float omega = 1; // rad/s
	r_deg_hp = (1 - omega * dt) * r_deg_hp + r_deg - r_deg_prev;
	r_deg_prev = r_deg;
	// UNUSED

	// Prepare the error vector
	std::vector<float> vec = {
		phi_err_deg,
		phi_err_i,
		p_deg,
		r_deg
	};

	// Compute the controller output
	std::vector<float> res = matmul(vec);
	if (res.size() != 2)
		fprintf(stderr, "Invalid output size: %lu\n", res.size());
	float ail = res[0] * speed_scaler;
	float rud = res[1] * speed_scaler;

	// Add feedforward
	ail += ControllerData::rollYaw.ailFF;
	rud += ControllerData::rollYaw.rudFF;

	// Clamp the output
	float ail_clamped = CLAMP(ail, -ControllerData::rollYaw.maxAilDeg, ControllerData::rollYaw.maxAilDeg);
	float rud_clamped = CLAMP(rud, -ControllerData::rollYaw.maxRudDeg, ControllerData::rollYaw.maxRudDeg);
	ail_saturation = sgn(ail - ail_clamped);
	rud_saturation = sgn(rud - rud_clamped);

	// Assign the output
	int16_t ail_cd = (int16_t)(ail_clamped * 100);
	int16_t rud_cd = (int16_t)(rud_clamped * 100);
	SRV_Channels::move_servo(SRV_Channel::k_aileron, ail_cd, -2500, 2500);
	SRV_Channels::move_servo(SRV_Channel::k_rudder,  -rud_cd, -1500, 1500);
	printf("phi_cmd %.2f, phi %.2f, ail %.2f, rud %.2f\n", phi_cmd_deg, phi_deg, ail_clamped, rud_clamped);
	// printf("phi_cmd %.2f, phi %.2f, phi_err %.2f, phi_err_i: %.2f, p %.2f, ail %.2f, rud %.2f\n", phi_cmd_deg, phi_deg, phi_err_deg, phi_err_i, p_deg, ail, rud);
}
