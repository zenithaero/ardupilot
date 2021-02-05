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
	float tas = MAX(5.f, table.tas_value[0]); // TODO: Extract in config
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

	printf("accel %.4f, %.4f, %.4f, %.4f, %.4f, %.4f\n", vec[0], vec[1], vec[2], vec[3], vec[4], vec[5]);

	// Compute the controller output
	std::vector<float> res = matmul(table.K, vec);
	const size_t n_act = DIM(ControllerData::alloc.max);
	if (soft_assert(res.size() == n_act, "Invalid output size %lu\n", res.size()))
		return;

	// Denormalize output
	for (size_t i = 0; i < res.size(); i++) {
		res[i] *= qbarInv;
	}
	printf("qbarinv %.4f\n", qbarInv);
	printf("res %.4f, %.4f, %.4f, %.4f, %.4ff\n", res[0], res[1], res[2], res[3], res[4]);

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
	printf("a_max %.4f, %.4f, %.4f, %.4f, %.4f, %.4f\n", a_max[0], a_max[1], a_max[2], a_max[3], a_max[4], a_max[5]);

	// Add feedforward
	float thr_left_ff = table.tas_interp->get(ControllerData::trim.thrLeft, DIM(ControllerData::trim.thrLeft), table.tas_value);
	float thr_right_ff = table.tas_interp->get(ControllerData::trim.thrRight, DIM(ControllerData::trim.thrRight), table.tas_value);
	float ail_ff = table.tas_interp->get(ControllerData::trim.ailDeg, DIM(ControllerData::trim.ailDeg), table.tas_value);
	float elev_ff = table.tas_interp->get(ControllerData::trim.elevDeg, DIM(ControllerData::trim.elevDeg), table.tas_value);
	float rud_ff = table.tas_interp->get(ControllerData::trim.rudDeg, DIM(ControllerData::trim.rudDeg), table.tas_value);

	// Set commands
	thr_left_cmd = res[0] + thr_left_ff;
	thr_right_cmd = res[1] + thr_right_ff;
	ail_cmd = res[2] + ail_ff;
	elev_cmd = res[3] + elev_ff;
	rud_cmd = res[4] + rud_ff + rudder_cmd_deg;

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

	// Log commands
	log.thr_left = thr_left_cmd;
	log.thr_right = thr_right_cmd;
	log.ail = ail_cmd;
	log.elev = elev_cmd;
	log.rud = rud_cmd;

	// Log feedforwards
	log.thr_left_ff = thr_left_ff;
	log.thr_right_ff = thr_right_ff;
	log.ail_ff = ail_ff;
	log.elev_ff = elev_ff;
	log.rud_ff = rud_ff;
}