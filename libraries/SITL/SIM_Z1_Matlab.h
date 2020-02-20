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

    /*
      reply packet sent from Matlab to ArduPilot
     */
    struct __attribute__ ((packed)) fdm_packet {
      float time;  // in seconds
      double home_lla[3];
      float accel_b[3];
      float vel_b[3];
      float vel_e[3];
      float pos_e[3];
      float pqr[3];
      float quat[4];
      float euler[3];
      float thr;
    };
private:
    /*
      packet sent to Matlab
     */
    struct __attribute__ ((packed)) servo_packet {
      // size matches sitl_input upstream
      float servos[16];
    };

    void recv_fdm(const struct sitl_input &input);
    void send_servos(const struct sitl_input &input);
    void drain_sockets();

    double last_timestamp;

    SocketAPM socket_sitl;
    const char *_Z1_Matlab_address = "127.0.0.1";
    int _Z1_Matlab_port = -1;
    static const uint64_t Z1_Matlab_TIMEOUT_US = 5e6;
};

}  // namespace SITL
