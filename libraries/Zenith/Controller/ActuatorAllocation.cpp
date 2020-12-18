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
	qbar = MAX(qbar, 1e-3);

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
	// TODO Zenith: update magic number 5
	if (soft_assert(res.size() == 5, "Invalid output size %lu\n", res.size()))
		return;

	// Res gives allocation

	// Compute max accelerations to avoid exceeding values
	
	

	
}