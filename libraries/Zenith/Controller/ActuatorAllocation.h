#pragma once

#include "ControllerCommon.h"

// Use linear controller framework for grid interpolation
class AllocationTable: public LinearController {
public:
    AllocationTable(AP_AHRS &ahrs);

    void update();
    using LinearController::K;
    using LinearController::tas_value;
    using LinearController::tas_interp;
};

class ActuatorAllocation {
public:
    ActuatorAllocation(AP_AHRS &ahrs);

    ~ActuatorAllocation();

    /* Do not allow copies */
    ActuatorAllocation(const ActuatorAllocation &other) = delete;
    ActuatorAllocation &operator=(const ActuatorAllocation&) = delete;

    void allocate(const Accel &accel, float rudder_cmd_deg);

    // Actuator flags
    bool enable_throttle;
    bool enable_attitude_long;
    bool enable_attitude_lat;

    // Last commands
    float thr_left_cmd;
    float thr_right_cmd;
    float ail_cmd;
    float elev_cmd;
    float rud_cmd;
    // 
    Accel accel_max;
    log_Alloc log;

protected:
	AP_AHRS &ahrs;
    AllocationTable table;
};
