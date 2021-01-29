/**
 * @brief Zenith controller
 * @date Created August 08, 2020
 * @author Bertrand Bevillard <bertrand@zenithaero.com>
 */

#include <AP_HAL/AP_HAL.h>
#include "ZenithController.h"

extern const AP_HAL::HAL& hal;

ZenithController::ZenithController(AP_AHRS &_ahrs) :
	ahrs(_ahrs),
	pitch_controller(_ahrs),
	roll_yaw_controller(_ahrs),
	spd_alt_controller(_ahrs),
	actuator_allocation(_ahrs) {
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


// Main 400Hz entry point
void ZenithController::update(float tas_cmd,
							  float h_cmd,
							  float theta_cmd_deg,
							  float roll_cmd_deg,
							  float rudder_cmd_deg) {
	// Reset active bitmask & log ahrs
	active_logs = 0;
	log_ahrs();

	actuator_allocation.enable_throttle = true;
	actuator_allocation.enable_attitude_long = true;
	actuator_allocation.enable_attitude_lat = true;

    if (actuator_allocation.enable_throttle) {
    	update_spd_alt(tas_cmd, h_cmd);
	}

	if (actuator_allocation.enable_attitude_long) {
		// Override
		theta_cmd_deg = 0.69f;
		stabilize_pitch(theta_cmd_deg);
	}

	if (actuator_allocation.enable_attitude_lat) {
		// Override
		roll_cmd_deg = 0.f;
		stabilize_rollyaw(roll_cmd_deg, rudder_cmd_deg);
	}

	// Now run allocator
	// TODO: handle doublets
	Accel accel_cmd(
		Vector3f(spd_alt_controller.ax_command, 0.f, 0.f),
		Vector3f(
			roll_yaw_controller.rx_command,
			pitch_controller.ry_command,
			roll_yaw_controller.rz_command
		)
	);
	actuator_allocation.allocate(accel_cmd, rudder_cmd_deg);
}


void ZenithController::stabilize_pitch(float theta_cmd_deg) {
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
	bool elev_doublet_active = elev_doublet.udpate(actuator_allocation.elev_cmd);

	// Stabilize pitch, roll & yaw
	if (!elev_doublet_active) {
		active_logs |= ZenithController::PITCH_MASK;
		pitch_controller.update(theta_cmd_deg, actuator_allocation.accel_max);
	}
}

void ZenithController::stabilize_rollyaw(float phi_cmd_deg, float rudder_deg) {
	// Execute doublets if needed
	bool ail_doublet_active = ail_doublet.udpate(actuator_allocation.ail_cmd);
	bool rud_doublet_active = rud_doublet.udpate(actuator_allocation.rud_cmd);

	if (!ail_doublet_active && !rud_doublet_active) {
		active_logs |= ZenithController::ROLLYAW_MASK;
		roll_yaw_controller.update(phi_cmd_deg, actuator_allocation.accel_max);
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

	spd_alt_controller.update(tas_cmd, h_cmd, actuator_allocation.accel_max);
}

void ZenithController::log_ahrs() {
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
}


void ZenithController::write_logs(AP_Logger &logger) {
	// Log controls
    if (active_logs & ZenithController::AHRS_MASK)
        logger.Write_CTRL(log);
    if (active_logs & ZenithController::PITCH_MASK)
        logger.Write_CTRL(pitch_controller.log);
    if (active_logs & ZenithController::ROLLYAW_MASK)
        logger.Write_CTRL(roll_yaw_controller.log);
    if (active_logs & ZenithController::SPDALT_MASK)
        logger.Write_CTRL(spd_alt_controller.log);
}