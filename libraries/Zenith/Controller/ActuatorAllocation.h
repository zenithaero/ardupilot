#pragma once

#include "ControllerCommon.h"

// Use linear controller framework for grid interpolation
class AllocationTable: public LinearController {
public:
    AllocationTable(AP_AHRS &ahrs);

    void update();
    using LinearController::tas_filter;
    using LinearController::K;
};

class ActuatorAllocation {
public:
    ActuatorAllocation(AP_AHRS &ahrs);

    ~ActuatorAllocation();

    /* Do not allow copies */
    ActuatorAllocation(const ActuatorAllocation &other) = delete;
    ActuatorAllocation &operator=(const ActuatorAllocation&) = delete;

protected:
	AP_AHRS &ahrs;
    AllocationTable table;

    void allocate(Vector3f linAccel, Vector3f rotAccel);
};
