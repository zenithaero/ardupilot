/**
 * @brief SpdAltController
 * @date Created August 08, 2020
 * @author Bertrand Bevillard <bertrand@zenithaero.com>
 */

#include "SpdAltController.h"
#include "ControllerCommon.cpp"

const char* spdAltFields[] = {"hErr", "hErrInt", "tasErr", "tasErrInt", "hDotErr", "<thetaDeg>", "<qDeg>"};

SpdAltController::SpdAltController(AP_AHRS &_ahrs)
	: LinearController(
		_ahrs,
		ControllerData::spdAlt.stateNames,
		spdAltFields,
		ControllerData::spdAlt.Ktas,
		ControllerData::spdAlt.K
	) {};

void SpdAltController::update(float tas_cmd, float h_cmd, const Accel &accel_max) {
	LinearController::update();

	// Retreive measurements
	float h;
	ahrs.get_relative_position_D_home(h);
	h *= -1;
	Vector3f vel;
	if (!ahrs.get_velocity_NED(vel))
		vel.z = 0;
	float h_dot = -vel.z;
	float theta_deg = ahrs.pitch_sensor / 100.f;
	float theta_trim = tas_interp->get(ControllerData::trim.theta, DIM(ControllerData::trim.theta), tas_value);
	float theta_trim_deg = degrees(theta_trim);
	theta_deg -= theta_trim_deg;
	float q_deg = ToDeg(ahrs.get_gyro().y);
	float eas;
	soft_assert(ahrs.airspeed_estimate(eas), "Estimated airspeed error\n");
	float tas = eas * ahrs.get_EAS2TAS();

	// Compute errors
	float h_err = h_cmd - h;
	float tas_err = tas_cmd - tas;
	float h_dot_err = 0 - h_dot;
	h_err = CLAMP(h_err, -ControllerData::spdAlt.maxAltErr, ControllerData::spdAlt.maxAltErr);
	tas_err = CLAMP(tas_err, -ControllerData::spdAlt.maxTasErr, ControllerData::spdAlt.maxTasErr);

	// Handle integrators
	if (dt > 0.1) {
		h_err_i = 0;
		tas_err_i = 0;
		h_err = 0;
		tas_err = 0;
	}
	float h_di = h_err * dt;
	float tas_di = tas_err * dt;
	// Anti-windup - Make sure the index is appropriate
	// TODO: figure out MIMO case - disabled until then
	if (sgn(h_di * K[1][2]) != sgn(pitch_saturation) || true)
		h_err_i += h_di;
	if (sgn(tas_di * K[0][4]) != sgn(ax_saturation) || true)
		tas_err_i += tas_di;

	// Clamp the integrator
	float max_i_h = MIN(
		accel_max.lin.x / MAX(1e-3, abs(K[0][1])),
		ControllerData::pitch.maxCmdDeg / MAX(1e-3, abs(K[1][1]))
	);
	float max_i_tas = MIN(
		accel_max.lin.x / MAX(1e-3, abs(K[0][3])),
		ControllerData::pitch.maxCmdDeg / MAX(1e-3, abs(K[1][3]))
	);
	h_err_i = CLAMP(h_err_i, -max_i_h, max_i_h);
	tas_err_i = CLAMP(tas_err_i, -max_i_tas, max_i_tas);

	// Prepare the error vector
	std::vector<float> vec = {
		h_err,
		h_err_i,
		tas_err,
		tas_err_i,
		h_dot_err,
		theta_deg,
		q_deg
	};

	// Compute the controller output
	std::vector<float> res = matmul(K, vec);
	if (soft_assert(res.size() == 2, "Invalid output size %lu\n", res.size()))
		return;
	float ax = res[0];
	float pitch = res[1];

	// Add feedforward
	pitch += theta_trim_deg;

	// Clamp the output
	float ax_clamped = CLAMP(ax, -accel_max.lin.x, accel_max.lin.x);
	float pitch_clamped = CLAMP(pitch, -ControllerData::pitch.maxCmdDeg, ControllerData::pitch.maxCmdDeg);
	ax_saturation = sgn(ax - ax_clamped);
	pitch_saturation = sgn(pitch - pitch_clamped);

	// Assign the output
	pitch_command = pitch_clamped;
	ax_command = ax_clamped;
	// printf("tas_cmd: %.2f, tas: %.2f, tas_err: %.2f, tas_err_i %.2f, tas_err_max_i: %.2f, thr_cmd: %.2f\n", tas_cmd, tas, tas_err, tas_err_i, max_i_tas, thr_command);
	// printf("h_cmd: %.2f, h: %.2f, h_err: %.2f, h_err_i %.2f, h_err_max_i: %.2f, pitch_cmd: %.2f\n", h_cmd, h, h_err, h_err_i, max_i_h, pitch_command);

	// Log output
	log.h_cmd = h_cmd;
	log.tas_cmd = tas_cmd;
	log.h_err = h_err;
	log.tas_err = tas_err;
	log.h_err_i = h_err_i;
	log.tas_err_i = tas_err_i;
	log.max_i_h = max_i_h;
	log.max_i_tas = max_i_tas;
	log.pitch_ff = theta_trim_deg;
	log.pitch_cmd = pitch_command;
	log.ax_cmd = ax_command;
}