/**
 * @brief Simulator connection for ASWING
 * @date Created Jan 16, 2020
 * @author Bertrand Bevillard <bertrand@zenithaero.com>
 */

#include "SIM_ASWING.h"

#include <stdio.h>

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL &hal;

namespace SITL
{

ASWING::ASWING(const char *frame_str) : Aircraft(frame_str),
                                        last_timestamp(0),
                                        sock(true)
{
  // try to bind to a specific port so that if we restart ArduPilot
  // ASWING keeps sending us packets. Not strictly necessary but
  // useful for debugging
  sock.bind("127.0.0.1", 9003);

  sock.reuseaddress();
  sock.set_blocking(false);
}

/*
 * decode and send servos for fixed wing
 */
void ASWING::send_servos(const struct sitl_input &input)
{
  float roll_rate = ((input.servos[0] - 1000) / 1000.0) - 0.5;
  float pitch_rate = ((input.servos[1] - 1000) / 1000.0) - 0.5;
  float yaw_rate = ((input.servos[3] - 1000) / 1000.0) - 0.5;
  float throttle = ((input.servos[2] - 1000) / 1000.0);

  servo_packet pkt;
  pkt.roll_rate = constrain_float(roll_rate, -0.5, 0.5);
  pkt.pitch_rate = constrain_float(pitch_rate, -0.5, 0.5);
  pkt.throttle = constrain_float(throttle, 0, 1);
  pkt.yaw_rate = constrain_float(yaw_rate, -0.5, 0.5);
  pkt.col_pitch = 0;

  sock.sendto(&pkt, sizeof(pkt), "127.0.0.1", 9002);
}

/*
 * receive an update from the FDM
 * This is a blocking function
 */
void ASWING::recv_fdm(const struct sitl_input &input)
{
  fdm_packet pkt;

  /*
   * we re-send the servo packet every 0.1 seconds until we get a
   * reply. This allows us to cope with some packet loss to the FDM
   */
  while (sock.recv(&pkt, sizeof(pkt), 100) != sizeof(pkt))
  {
    send_servos(input);
  }

  accel_body = Vector3f(pkt.xAccel, pkt.yAccel, pkt.zAccel);
  gyro = Vector3f(pkt.rollRate, pkt.pitchRate, pkt.yawRate);
  velocity_ef = Vector3f(pkt.speedN, pkt.speedE, pkt.speedD);

  Location loc1, loc2;
  loc2.lat = pkt.latitude * 1.0e7;
  loc2.lng = pkt.longitude * 1.0e7;
  const Vector2f posdelta = loc1.get_distance_NE(loc2);
  position.x = posdelta.x;
  position.y = posdelta.y;
  position.z = -pkt.altitude;

  airspeed = pkt.airspeed;
  airspeed_pitot = pkt.airspeed;

  dcm.from_euler(pkt.roll, pkt.pitch, pkt.yaw);

  // auto-adjust to ASWING frame rate
  double deltat = pkt.timestamp - last_timestamp;
  time_now_us += deltat * 1.0e6;

  if (deltat < 0.01 && deltat > 0)
  {
    adjust_frame_time(1.0 / deltat);
  }
  last_timestamp = pkt.timestamp;
}

/*
 * update the ASWING simulation by one time step
 */
void ASWING::update(const struct sitl_input &input)
{
  send_servos(input);
  recv_fdm(input);
  update_position();
  time_advance();

  // update magnetic field
  update_mag_field_bf();
}

} // namespace SITL
