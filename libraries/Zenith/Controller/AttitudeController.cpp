/**
 * @brief AttitudeController
 * @date Created August 08, 2020
 * @author Bertrand Bevillard <bertrand@zenithaero.com>
 */

#include "AttitudeController.h"

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
	theta_err_i = 0;
}

void PitchController::update(float theta_cmd_deg) {
	LinearController::update();

	// Compute speed scaler
	float speed_scaler = 1;
	float airspeed;
    if (ahrs.airspeed_estimate(airspeed)) {
		airspeed = MAX(1, airspeed);
		speed_scaler = ControllerData::constants.scalingSpeed / airspeed;
		speed_scaler = CLAMP(speed_scaler, 0.2, 2);
    }

	float theta_deg = ahrs.pitch_sensor / 100.f;
	float theta_err_deg = theta_cmd_deg - theta_deg;
	theta_err_deg = CLAMP(theta_err_deg, -ControllerData::pitch.maxCmdDeg, ControllerData::pitch.maxCmdDeg);

	// Handle integrator
	if (dt > 0.1) {
		theta_err_i = 0;
		theta_err_deg = 0;
	}
	float di = theta_err_deg * dt;
	// Anti-windup - Make sure the index is appropriate
	if (sgn(di * K[0][1]) != sgn(elev_saturation) || true)
		theta_err_i += di;

	// Clamp the integrator
	float max_i = ControllerData::pitch.maxIntCmdDeg / MAX(1e-3, abs(K[0][1]));
	theta_err_i = CLAMP(theta_err_i, -max_i, max_i);

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
	
	float elev = res[0] * speed_scaler;

	// Add feedforward
	float elev_ff = tas_interp->get(ControllerData::trim.elevDeg, DIM(ControllerData::trim.elevDeg), tas_value);
	elev += elev_ff;

	// Clamp the elevator
	elev_command = CLAMP(elev, -ControllerData::pitch.maxElevDeg, ControllerData::pitch.maxElevDeg);
	elev_saturation = sgn(elev - elev_command);

	// Assign the output
	// printf("theta_cmd %.2f, theta %.2f, theta_err %.2f, theta_err_i %.2f, max_i %.2f, elev_ff %.2f, elev %.2f\n", theta_cmd_deg, theta_deg, theta_err_deg, theta_err_i, max_i, elev_ff, elev_command);
	int16_t elev_cd = (int16_t)(elev_command * 100);
	// SRV_Channels::move_servo(SRV_Channel::k_elevator, elev_cd, -1500, 1500); // doesn't seem to work with mixing
	SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, elev_cd);
	// SRV_Channels:set_scaled

	// Log values
	log.theta_cmd_deg = theta_cmd_deg;
	log.theta_err_deg = theta_err_deg;
	log.theta_err_i = theta_err_i;
	log.max_i = max_i;
	log.elev_ff = elev_ff;
	log.elev_cmd = elev_command;
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

void RollYawController::update(float phi_cmd_deg, float rudder_deg) {
	LinearController::update();

	// Compute speed scaler
	float speed_scaler = 1;
	float airspeed;
    if (ahrs.airspeed_estimate(airspeed)) {
		airspeed = MAX(1, airspeed);
		speed_scaler = ControllerData::constants.scalingSpeed / airspeed;
		speed_scaler = CLAMP(speed_scaler, 0.2, 2);
    }

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
	if (sgn(di * K[0][1]) != sgn(ail_saturation) || true)
		phi_err_i += di;

	// Clamp the integrator
	float max_i = MIN(
		ControllerData::rollYaw.maxIntCmdDeg / MAX(1e-3, abs(K[0][1])),
		ControllerData::rollYaw.maxIntCmdDeg / MAX(1e-3, abs(K[1][1]))
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
	float ail = res[0] * speed_scaler;
	float rud = res[1] * speed_scaler;

	// Add feedforward
	float ail_ff = tas_interp->get(ControllerData::trim.ailDeg, DIM(ControllerData::trim.ailDeg), tas_value);
	ail += ail_ff;
	float rud_ff = tas_interp->get(ControllerData::trim.rudDeg, DIM(ControllerData::trim.rudDeg), tas_value);
	rud += rud_ff;

	// Add manual rudder input
	rud += rudder_deg;

	// Clamp the output
	ail_command = CLAMP(ail, -ControllerData::rollYaw.maxAilDeg, ControllerData::rollYaw.maxAilDeg);
	rud_command = CLAMP(rud, -ControllerData::rollYaw.maxRudDeg, ControllerData::rollYaw.maxRudDeg);
	ail_saturation = sgn(ail - ail_command);
	rud_saturation = sgn(rud - rud_command);

	// Assign the output
	int16_t ail_cd = (int16_t)(ail_command * 100);
	// SRV_Channels::move_servo(SRV_Channel::k_aileron, ail_cd, -2500, 2500); // DOESN'T SEEM TO WORK
	SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, ail_cd);
	int16_t rud_cd = (int16_t)(rud_command * 100);
	// SRV_Channels::move_servo(SRV_Channel::k_rudder,  -rud_cd, -1500, 1500); // DOESN'T SEEM TO WORK
	SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, rud_cd);
	// printf("phi_cmd %.2f, phi %.2f, ail %.2f, rud %.2f\n", phi_cmd_deg, phi_deg, ail_command, rud_command);

	// Log values
	log.phi_cmd_deg = phi_cmd_deg;
	log.phi_err_deg = phi_err_deg;
	log.phi_err_i = phi_err_i;
	log.max_i = max_i;
	log.ail_ff = ail_ff;
	log.rud_ff = rud_ff;
	log.ail_cmd = ail_command;
	log.rud_cmd = rud_command;
}