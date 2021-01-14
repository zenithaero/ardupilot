/**
 * @brief ActuatorAllocation
 * @date Created August 08, 2020
 * @author Bertrand Bevillard <bertrand@zenithaero.com>
 */

#include "ActuatorAllocation.h"
#include "ControllerCommon.cpp"

const char* spdAltFields[] = {"Ax", "Ay", "Az", "Rx", "Ry", "Rz"};

AllocationTable::AllocationTable(AP_AHRS &_ahrs)
	: LinearController(
		_ahrs,
		spdAltFields,
		spdAltFields,
		ControllerData::alloc.tas,
		ControllerData::alloc.table
	) {};

void AllocationTable::update() {
	LinearController::update();
}

ActuatorAllocation::ActuatorAllocation(AP_AHRS &_ahrs) : ahrs(_ahrs), table(ahrs) {}

ActuatorAllocation::~ActuatorAllocation() {}

void ActuatorAllocation::allocate(const Accel &accel, float rudder_cmd_deg) {
	// Update table
	table.update();

	// Compute qbar
	float tas = MAX(10.f, table.tas_value[0]); // TODO: Extract in config
	float qbar = tas * tas * 0.5 * 1.225;
	float qbarInv = 1.f / qbar;

	// Prepare the command vector
	std::vector<float> vec = {
		accel.lin.x,
		accel.lin.y,
		accel.lin.z,
		accel.rot.x,
		accel.rot.y,
		accel.rot.z
	};

	// Compute the controller output
	std::vector<float> res = matmul(table.K, vec);
	const size_t n_act = DIM(ControllerData::alloc.max);
	if (soft_assert(res.size() == n_act, "Invalid output size %lu\n", res.size()))
		return;

	// Denormalize output
	for (size_t i = 0; i < vec.size(); i++) {
		vec[i] *= qbarInv;
	}

	// Compute max accelerations to avoid exceeding values
	float a_max[6];
	for (size_t i = 0; i < vec.size(); i ++) {
		float min_val = INFINITY;
		for (size_t j = 0; j < n_act; j++) {
			float alloc_max = ControllerData::alloc.max[j];
			float kij =  fabsf(table.K[j][i]) * qbarInv;
			float sub_min = alloc_max / MAX(FLT_EPSILON, kij);
			min_val = MIN(min_val, sub_min);
		}
		a_max[i] = min_val;
	}
	accel_max.lin = Vector3f(a_max[0], a_max[1], a_max[2]);
	accel_max.rot = Vector3f(a_max[3], a_max[4], a_max[5]);

	// Add feedforward
	float thr_left_ff = table.tas_interp->get(ControllerData::trim.thrLeft, DIM(ControllerData::trim.thrLeft), table.tas_value);
	float thr_right_ff = table.tas_interp->get(ControllerData::trim.thrRight, DIM(ControllerData::trim.thrRight), table.tas_value);
	float thr_ail_ff = table.tas_interp->get(ControllerData::trim.ailDeg, DIM(ControllerData::trim.ailDeg), table.tas_value);
	float thr_elev_ff = table.tas_interp->get(ControllerData::trim.elevDeg, DIM(ControllerData::trim.elevDeg), table.tas_value);
	float thr_rud_ff = table.tas_interp->get(ControllerData::trim.rudDeg, DIM(ControllerData::trim.rudDeg), table.tas_value);

	// Set commands
	thr_left_cmd = res[0] + thr_left_ff;
	thr_right_cmd = res[1] + thr_right_ff;
	ail_cmd = res[2] + thr_ail_ff;
	elev_cmd = res[3] + thr_elev_ff;
	rud_cmd = res[4] + thr_rud_ff + rudder_cmd_deg;


	// printf("max int: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n", a_max[0], a_max[1], a_max[2], a_max[3], a_max[4], a_max[5]);
	// printf("ff: [%.2f, %.2f, %.2f, %.2f, %.2f]\n", thr_left_ff, thr_right_ff, thr_ail_ff, thr_elev_ff, thr_rud_ff);
	// printf("output: [%.2f, %.2f, %.2f, %.2f, %.2f]\n", thr_left_cmd, thr_right_cmd, ail_cmd, elev_cmd, rud_cmd);
	

	// Res gives allocation. Deal the allocation back to the actuators
	if (enable_throttle) {
		SRV_Channels::set_output_scaled(SRV_Channel::k_throttleLeft, (int16_t)(thr_left_cmd * 100));
		SRV_Channels::set_output_scaled(SRV_Channel::k_throttleRight, (int16_t)(thr_right_cmd * 100));
	}
	if (enable_attitude_long) {
		SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, (int16_t)(elev_cmd * 100));
	}
	if (enable_attitude_lat) {
		SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, (int16_t)(ail_cmd * 100));
		SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, (int16_t)(rud_cmd * 100));
	}
}