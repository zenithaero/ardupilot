#pragma once

#include "ControllerCommon.h"
#include "AttitudeController.h"
#include "SpdAltController.h"

class ZenithController {
public:
    ZenithController(AP_AHRS &_ahrs);

    /* Do not allow copies */
    ZenithController(const ZenithController &other) = delete;
    ZenithController &operator=(const ZenithController&) = delete;

    void update(
        bool update_spd_alt,
        float tas_cmd,
        float h_cmd,
        bool update_attitude,
        float theta_cmd_deg,
        float roll_cmd_deg,
        float rudder_deg
    );
    void log_ahrs();
    void stabilize_pitch(float theta_cmd_deg);
    void stabilize_rollyaw(float roll_cmd_deg, float rudder_deg);
    void update_spd_alt(float tas_cmd, float h_cmd);
    void write_logs(AP_Logger &logger);


    // Doublets
    DoubletHandler ail_doublet;
    DoubletHandler elev_doublet;
    DoubletHandler rud_doublet;

    // Controllers
    PitchController pitch_controller;
    RollYawController roll_yaw_controller;
    SpdAltController spd_alt_controller;

    // Log structures
    log_AhrsCtrl log;
    static const uint32_t AHRS_MASK = 0x1;
    static const uint32_t PITCH_MASK = 0x2;
    static const uint32_t ROLLYAW_MASK = 0x4;
    static const uint32_t SPDALT_MASK = 0x8;
    uint32_t active_logs = 0;

protected:
	AP_AHRS &ahrs;
    // CrossTrackController xtrack_controller{ahrs};

    // TEMP
    uint64_t t0 = 0;
};

