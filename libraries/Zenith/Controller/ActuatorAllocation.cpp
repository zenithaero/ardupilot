/**
 * @brief ActuatorAllocation
 * @date Created August 08, 2020
 * @author Bertrand Bevillard <bertrand@zenithaero.com>
 */

#include "ControllerCommon.h"
#include "ActuatorAllocation.h"

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

// float AllocationTable::tasFiltered() {
// 	return tas_filter.get();
// }

ActuatorAllocation::ActuatorAllocation(AP_AHRS &_ahrs) : ahrs(_ahrs), table(ahrs) {
	
}

ActuatorAllocation::~ActuatorAllocation() {
}

void ActuatorAllocation::allocate(Vector3f linAccel, Vector3f rotAccel) {
	// Update table
	table.update();

	// Compute qbar
	float tas = table.tas_filter.get();
	float qbar = tas * tas * 0.5 * 1.225;
	float qbarInv = 1.f / MAX(qbar, 1e-3);


	// Prepare the command vector
	std::vector<float> vec = {
		linAccel.x,
		linAccel.y,
		linAccel.z,
		rotAccel.x,
		rotAccel.y,
		rotAccel.z
	};

	// Compute the controller output
	std::vector<float> res = matmul(table.K, vec);
	const float n = DIM(ControllerData::alloc.max);
	if (soft_assert(res.size() == n, "Invalid output size %lu\n", res.size()))
		return;

	// Denormalize output
	for (size_t i = 0; i < vec.size(); i++) {
		vec[i] *= qbarInv;
	}

	// Compute max accelerations to avoid exceeding values
	for (size_t i = 0; i < vec.size(); i ++) {
		float max_val = 0.f;
		for (size_t j = 0; j < n; j++) {
			float alloc_max = ControllerData::alloc.max[j];
			float kij =  MAX(FLT_EPSILON, fabsf(table.K[j][i]) * qbarInv );
			float sub_max = alloc_max / kij;
			max_val = MAX(max_val, sub_max);
		}
		max_accel[i] = max_val;
	}

	// Res gives allocation. Deal the allocation back to the actuators
	// TODO Zenith: Make generic
	// 0 - throttle
	// 1 - throttle
	// 2 - ail
	// 3 - elev
	// 4 - rudder
	SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, (int16_t)(res[0] * 100));
	// SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, (int16_t)(res[1] * 100));
	SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, (int16_t)(res[2] * 100));
	SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, (int16_t)(res[3] * 100));
	SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, (int16_t)(res[4] * 100));
}