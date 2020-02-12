/**
 * @brief Z1 Matlab connection class
 * @date Created Feb 8, 2020
 * @author Bertrand Bevillard <bertrand@zenithaero.com>
 */

#pragma once

#include "SIM_Aircraft.h"
#include <AP_HAL/utility/Socket.h>

namespace SITL {

/*
  Z1_Matlab simulator
 */
class Z1_Matlab : public Aircraft {
public:
    Z1_Matlab(const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input) override;

    /* static object creator */
    static Aircraft *create(const char *frame_str) {
        return new Z1_Matlab(frame_str);
    }

    /*  Create and set in/out socket for Z1_Matlab simulator */
    void set_interface_ports(const char* address, const int port_in, const int port_out) override;

private:
    /*
      packet sent to Z1_Matlab
     */
    struct servo_packet {
      // size matches sitl_input upstream
      float servos[16];
    };

    /*
      reply packet sent from Z1_Matlab to ArduPilot
     */
    struct fdm_packet {
      float timestamp;  // in seconds
      float imu_angular_velocity_rpy[3];
      float imu_linear_acceleration_xyz[3];
      float imu_orientation_quat[4];
      float velocity_xyz[3];
      float position_xyz[3];
    };

    void recv_fdm(const struct sitl_input &input);
    void send_servos(const struct sitl_input &input);
    void drain_sockets();

    double last_timestamp;

    SocketAPM socket_sitl;
    const char *_Z1_Matlab_address = "127.0.0.1";
    int _Z1_Matlab_port = 9002;
    static const uint64_t Z1_Matlab_TIMEOUT_US = 5000000;
};

}  // namespace SITL
