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

ZenithController::ZenithController(AP_AHRS &_ahrs) :
	ahrs(_ahrs),
	pitch_controller(_ahrs),
	roll_yaw_controller(_ahrs),
	spd_alt_controller(_ahrs)  {
	// Init doublet handlers (channel, deflection, duration, settle_time)
	ail_doublet = DoubletHandler(SRV_Channel::k_aileron, 2, 1, 5); 
	elev_doublet = DoubletHandler(SRV_Channel::k_elevator, 2, 1, 5);
	rud_doublet = DoubletHandler(SRV_Channel::k_rudder, 2, 1, 5);
};

void DoubletHandler::start() {
	printf("Started doublet\n");
	t_start = AP_HAL::micros64();
}

void DoubletHandler::clear() {
	printf("Cleared doublet\n");
	t_start = 0;
}

bool DoubletHandler::udpate(float last_command) {
	uint64_t now = AP_HAL::micros64();
	float dt = (now - t_start) / 1e6f;
	bool active = t_start != 0 && dt < duration + settle_time;
	if (!active) {
		command_filter.set_cutoff_frequency(1.f / 5); // T = 5s
		command_filter.apply(last_command, dt);
	} else {
		// Apply doublet
		float command = command_filter.get();
		if (dt < duration)
			command += deflection;
		SRV_Channels::set_output_scaled(channel, (int16_t)(command * 100));
	}
	return active;
}

void ZenithController::update() {
	// Reset active bitmask
	active_logs = 0;

	// Log ahrs
	Vector3f vel, pos;
	bool valid_ahrs = true;
    valid_ahrs &= ahrs.get_velocity_NED(vel);
    Vector3f gyro = ahrs.get_gyro();
    valid_ahrs &= ahrs.get_relative_position_NED_home(pos);
    auto acc = AP::ins().get_accel();
	if (valid_ahrs) {
		log.phi = ahrs.roll_sensor;
		log.theta = ahrs.pitch_sensor;
		log.psi = ahrs.yaw_sensor;
		log.gx = gyro.x;
		log.gy = gyro.y;
		log.gz = gyro.z;
		log.vn = vel.x;
		log.ve = vel.y;
		log.vd = vel.z;
		log.pn = pos.x;
		log.pe = pos.y;
		log.pd = pos.z;
	}


    // AP::logger().Write("AHRS", "TimeUS,gx,gy,gz,vn,ve,vd,n,e,d,ax,ay,az", "Qffffffffffff",
    //     AP_HAL::micros64(),
    //     gyro.x, gyro.y, gyro.z,
    //     vel.x, vel.y, vel.z,
    //     pos.x, pos.y, pos.z,
    //     acc.x, acc.y, acc.z);
}


void ZenithController::stabilize(float theta_cmd_deg, float phi_cmd_deg, float rudder_deg) {
	// Full stabilisation, or input (/state) based mode?
	// Ignore ardupilot state machine?

	// Compute speed scaler
	float speed_scaler = 1;
	float airspeed;
    if (ahrs.airspeed_estimate(airspeed)) {
		airspeed = MAX(1, airspeed);
		speed_scaler = ControllerData::constants.scalingSpeed / airspeed;
		speed_scaler = CLAMP(speed_scaler, 0.2, 2);
    }

	// TEMP
	// if (t0 == 0)
    //     t0 = AP_HAL::micros64();
    // float dt = (AP_HAL::micros64() - t0) / 1e6f;

	// phi_cmd_deg = 0;
	// theta_cmd_deg = degrees(0.00557);
	// if (dt > 70)
	// 	theta_cmd_deg += 5;
	// else {
	// 	// reset integrators
	// 	pitch_controller.reset();
	// }
	// if (dt > 90)
	// 	exit(-1);

	// Execute doublets if needed
	bool ail_doublet_active = ail_doublet.udpate(roll_yaw_controller.ail_command);
	bool elev_doublet_active = elev_doublet.udpate(pitch_controller.elev_command);
	bool rud_doublet_active = rud_doublet.udpate(roll_yaw_controller.rud_command);

	// Stabilize pitch, roll & yaw
	if (!elev_doublet_active) {
		active_logs |= ZenithController::PITCH_MASK;
		pitch_controller.update(theta_cmd_deg, speed_scaler);
	}
	if (!ail_doublet_active && !rud_doublet_active) {
		active_logs |= ZenithController::ROLLYAW_MASK;
		roll_yaw_controller.update(phi_cmd_deg, rudder_deg, speed_scaler);
	}
}

void ZenithController::update_spd_alt(float tas_cmd, float h_cmd) {
	active_logs |= ZenithController::SPDALT_MASK;
	// Step command

	// TEMP
	// h_cmd = 0;
	// tas_cmd = 15;

	// float dt = (AP_HAL::micros64() - t0) / 1e6f;
	// if (dt > 90)
	// 	h_cmd = 10;
	// if (dt > 130)
	// 	tas_cmd = 20;
	// if (dt > 160)
	// 	exit(-1);

	spd_alt_controller.update(tas_cmd, h_cmd);
}

template<size_t n, size_t n_tas, size_t n_k>
LinearController::LinearController(
	AP_AHRS &_ahrs,
	const char* (&stateNames)[n],
	const char* (&expectedNames)[n],
	const float (&_K_tas)[n_tas],
	const float (&_K_grid)[n_k]) : ahrs(_ahrs) {
	// Control state names
	for (size_t j = 0; j < n; j++)
		hard_assert(strcmp(stateNames[j], expectedNames[j]) == 0,
			"State mismatch: %s (expected %s)\n", stateNames[j], expectedNames[j]);

	// Create tas interpolation
	std::vector<float> tas_lookup(
		ControllerData::trim.tas,
		ControllerData::trim.tas + DIM(ControllerData::trim.tas)
	);
    std::vector<std::vector<float>> interp = { tas_lookup };
	tas_interp = new Interp<float>(interp);

	// Create K interpolation
	this->K_grid = std::vector<float>(_K_grid, _K_grid + n_k);
	std::vector<float> K_tas(_K_tas, _K_tas + n_tas);
    interp = {K_tas};
	K_interp = new Interp<float>(interp);
	
	// Initialize gain matrix
	size_t m = n_k / n / n_tas;
	hard_assert(m * n * n_tas == n_k,
		"The grid length (%lu) must equal m (%lu) * n (%lu) * n_tas (%lu)\n",
		n_k, m, n, n_tas);
	for (size_t i = 0; i < m; i++)
		K.push_back(std::vector<float>(n, 0));
}

LinearController::~LinearController() {
	delete(tas_interp);
	delete(K_interp);
}

void LinearController::update() {
	// Update timestep
	uint64_t t_now = AP_HAL::micros64();
	dt = (float)(t_now - t_prev) / 1e6f;
	t_prev = t_now;

	// Update tas filter
	float airspeed;
    if (ahrs.airspeed_estimate(airspeed)) {
		tas_filter.set_cutoff_frequency(1.f / 5); // T = 5s
		tas_filter.apply(airspeed, dt);
	}

	// Update controller gains
	tas_value[0] = tas_filter.get();
	std::vector<float> vec = K_interp->get_vec(K_grid, tas_value);
	reshape(vec, K);
}

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

void PitchController::update(float theta_cmd_deg, float speed_scaler) {
	LinearController::update();

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
	printf("theta_cmd %.2f, theta %.2f, theta_err %.2f, theta_err_i %.2f, max_i %.2f, elev_ff %.2f, elev %.2f\n", theta_cmd_deg, theta_deg, theta_err_deg, theta_err_i, max_i, elev_ff, elev_command);
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

void RollYawController::update(float phi_cmd_deg, float rudder_deg, float speed_scaler) {
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

const char* spdAltFields[] = {"hErr", "hErrInt", "tasErr", "tasErrInt", "hDotErr", "<thetaDeg>", "<qDeg>"};

SpdAltController::SpdAltController(AP_AHRS &_ahrs)
	: LinearController(
		_ahrs,
		ControllerData::spdAlt.stateNames,
		spdAltFields,
		ControllerData::spdAlt.Ktas,
		ControllerData::spdAlt.K
	) {};

void SpdAltController::update(float tas_cmd, float h_cmd) {
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
	if (sgn(h_di * K[1][2]) != sgn(pitch_saturation) || true)
		h_err_i += h_di;
	if (sgn(tas_di * K[0][4]) != sgn(thr_saturation) || true)
		tas_err_i += tas_di;

	// Clamp the integrator
	float max_i_h = MIN(
		ControllerData::spdAlt.maxIntThrCmd / MAX(1e-3, abs(K[0][1])),
		ControllerData::spdAlt.maxIntThetaCmdDeg / MAX(1e-3, abs(K[1][1]))
	);
	float max_i_tas = MIN(
		ControllerData::spdAlt.maxIntThrCmd / MAX(1e-3, abs(K[0][3])),
		ControllerData::spdAlt.maxIntThetaCmdDeg / MAX(1e-3, abs(K[1][3]))
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
	float thr = res[0];
	float pitch = res[1];

	// Add feedforward
	float thr_ff = tas_interp->get(ControllerData::trim.thr, DIM(ControllerData::trim.thr), tas_value);
	thr += thr_ff;
	pitch += theta_trim_deg;

	// Clamp the output
	float thr_clamped = CLAMP(thr, 0, ControllerData::spdAlt.maxThr);
	float pitch_clamped = CLAMP(pitch, -ControllerData::pitch.maxCmdDeg, ControllerData::pitch.maxCmdDeg);
	thr_saturation = sgn(thr - thr_clamped);
	pitch_saturation = sgn(pitch - pitch_clamped);

	// Assign the output
	pitch_command = pitch_clamped;
	thr_command = thr_clamped;
	printf("tas_cmd: %.2f, tas: %.2f, tas_err: %.2f, tas_err_i %.2f, tas_err_max_i: %.2f, thr_cmd: %.2f\n", tas_cmd, tas, tas_err, tas_err_i, max_i_tas, thr_command);
	printf("h_cmd: %.2f, h: %.2f, h_err: %.2f, h_err_i %.2f, h_err_max_i: %.2f, pitch_cmd: %.2f\n", h_cmd, h, h_err, h_err_i, max_i_h, pitch_command);

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
	log.thr_ff = thr_ff;
	log.pitch_cmd = pitch_command;
	log.thr_cmd = thr_command;
}


// TODO: implement those
// void CrossTrackController::update_waypoint(const Location &prev_WP, const Location &next_WP, float dist_min) {
// 	Location loc;
// 	if (!ahrs.get_position(loc)) {
// 		// Maintain the last command
// 		return;
// 	}

// 	Vector2f gnd_spd = ahrs.groundspeed_vector();
	

// }

// void CrossTrackController::update_loiter(const Location &center_WP, float radius, int8_t loiter_direction) {

// }