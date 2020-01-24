/**
 * @brief Simulator connection for ASWING
 * @date Created Jan 16, 2020
 * @author Bertrand Bevillard <bertrand@zenithaero.com>
 */

#pragma once

#include <AP_HAL/utility/Socket.h>

#include "SIM_Aircraft.h"

namespace SITL
{

/*
 * ASWING simulator
 */
class ASWING : public Aircraft
{
public:
  ASWING(const char *frame_str);

  /* update model by one time step */
  void update(const struct sitl_input &input) override;

  /* static object creator */
  static Aircraft *create(const char *frame_str)
  {
    return new ASWING(frame_str);
  }

private:
  /*
   * packet sent to ASWING
   */
  struct servo_packet
  {
    float roll_rate;
    float pitch_rate;
    float throttle;
    float yaw_rate;
    float col_pitch;
  };

  /*
   * reply packet sent from ASWING to ArduPilot
   */
  struct fdm_packet
  {
    double timestamp;
    double latitude, longitude;
    double altitude;
    double heading;
    double speedN, speedE, speedD;
    double xAccel, yAccel, zAccel;
    double rollRate, pitchRate, yawRate;
    double roll, pitch, yaw;
    double airspeed;
  };

  void recv_fdm(const struct sitl_input &input);
  void send_servos(const struct sitl_input &input);

  double last_timestamp;
  SocketAPM sock;
};

} // namespace SITL
