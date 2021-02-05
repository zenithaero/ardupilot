/**
 * @brief AttitudeController
 * @date Created August 08, 2020
 * @author Bertrand Bevillard <bertrand@zenithaero.com>
 */

#include "AttitudeController.h"
#include "ControllerCommon.cpp"

const char* pitchFields[] = {"thetaErrDeg", "thetaErrInt", "<qDeg>"}; // , "<hDot>"},

PitchController::PitchController(AP_AHRS &_ahrs)
	: LinearController(
		_ahrs,
		ControllerData::pitch.stateNames,
		pitchFields,
		ControllerData::pitch.Ktas,
		ControllerData::pitch.K
	) {};

static uint32_t counter, counter2 = 0; // TEMP

void PitchController::reset() {
	theta_err_i = 0.f;
	ry_command = 0.f;
}

void PitchController::update(float theta_cmd_deg, const Accel &accel_max) {
	LinearController::update();
	float theta_deg = ahrs.pitch_sensor / 100.f;
	float theta_err_deg = theta_cmd_deg - theta_deg;
	printf("PITCH THETA CMD DEG: %.2f\n", theta_cmd_deg);
	theta_err_deg = CLAMP(theta_err_deg, -ControllerData::pitch.maxCmdDeg, ControllerData::pitch.maxCmdDeg);

	// Handle integrator
	if (dt > 0.1) {
		theta_err_i = 0;
		theta_err_deg = 0;
	}
	float di = theta_err_deg * dt;
	// Anti-windup - Make sure the index is appropriate
	if (sgn(di * K[0][1]) != sgn(ry_saturation) || true)
		theta_err_i += di;

	// Clamp the integrator
	float max_i = accel_max.rot.y / MAX(1e-3, abs(K[0][1]));
	theta_err_i = CLAMP(theta_err_i, -max_i, max_i);
	theta_err_i = 0.f;

	// Retreive q
	float q_deg = ToDeg(ahrs.get_gyro().y);
	// Vector3f vel;
	// if (!ahrs.get_velocity_NED(vel))
	// 	vel.z = 0;
	// float h_dot = -vel.z;

	// Retrieve and filter ax
	// float ax = -AP::ins().get_accel().x;
	// ax_filter.set_cutoff_frequency(1 / (M_PI * 2 * 0.1));
	// float ax_filt = ax_filter.apply(ax, dt);

	// Prepare the error vector
	std::vector<float> vec = {
		theta_err_deg,
		theta_err_i,
		q_deg
		// ax_filt
		// h_dot
	};

	// Compute the controller output
	std::vector<float> res = matmul(K, vec);
	if (soft_assert(res.size() == 1, "Invalid output size %lu\n", res.size()))
		return;
	
	float ry = res[0];

	// Clamp the ryator
	ry_command = CLAMP(ry, -accel_max.rot.y, accel_max.rot.y);
	ry_saturation = sgn(ry - ry_command);

	// Log values
	log.theta_cmd_deg = theta_cmd_deg;
	log.theta_err_deg = theta_err_deg;
	log.theta_err_i = theta_err_i;
	log.max_i = max_i;
	log.ry_cmd = ry_command;
}

const char* rollFields[] = {"phiErrDeg", "phiErrInt", "<pDeg>", "rDegHP"};

RollYawController::RollYawController(AP_AHRS &_ahrs)
	: LinearController(
		_ahrs,
		ControllerData::rollYaw.stateNames,
		rollFields,
		ControllerData::rollYaw.Ktas,
		ControllerData::rollYaw.K
	) {};

void RollYawController::reset() {
	phi_err_i = 0.f;
	rx_command = 0.f;
	rz_command = 0.f;
}

void RollYawController::update(float phi_cmd_deg, const Accel &accel_max) {
	LinearController::update();

	float phi_deg = ahrs.roll_sensor / 100.f;
	float phi_err_deg = phi_cmd_deg - phi_deg;
	phi_err_deg = CLAMP(phi_err_deg, -ControllerData::rollYaw.maxCmdDeg, ControllerData::rollYaw.maxCmdDeg);

	// Handle integrator
	if (dt > 0.1) {
		phi_err_i = 0;
		phi_err_deg = 0;
	}
	float di = phi_err_deg * dt;
	// Anti-windup - Make sure the index is appropriate
	if (sgn(di * K[0][1]) != sgn(rx_saturation) || true)
		phi_err_i += di;

	// Clamp the integrator
	float max_i = MIN(
		accel_max.rot.x / MAX(1e-3, abs(K[0][1])),
		accel_max.rot.z / MAX(1e-3, abs(K[1][1]))
	);
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
	std::vector<float> res = matmul(K, vec);
	if (soft_assert(res.size() == 2, "Invalid output size %lu\n", res.size()))
		return;
	float rx = res[0];
	float rz = res[1];

	// Clamp the output
	rx_command = CLAMP(rx, -accel_max.rot.x, accel_max.rot.x);
	rz_command = CLAMP(rz, -accel_max.rot.z, accel_max.rot.z);
	rx_saturation = sgn(rx - rx_command);
	rz_saturation = sgn(rz - rz_command);

	// Log values
	log.phi_cmd_deg = phi_cmd_deg;
	log.phi_err_deg = phi_err_deg;
	log.phi_err_i = phi_err_i;
	log.max_i = max_i;
	log.rx_cmd = rx_command;
	log.rz_cmd = rz_command;
}