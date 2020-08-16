/**
 * @brief Z1 wrapper class
 * @date Created Feb 12, 2020
 * @author Bertrand Bevillard <bertrand@zenithaero.com>
 */

#pragma once

#include "SIM_Aircraft.h"
#include <AP_HAL/utility/Socket.h>
#include "Generated/Simulator.h"

namespace SITL
{

/*
  Z1 simulator
 */
class Z1_Wrapper : public Aircraft
{
public:
  Z1_Wrapper(const char *frame_str);

  /* update model by one time step */
  void update(const struct sitl_input &input) override;

  /* static object creator */
  static Aircraft *create(const char *frame_str)
  {
    return new Z1_Wrapper(frame_str);
  }

private:
  void recv_fdm();
  void send_servos(const struct sitl_input &input);

  double last_timestamp;
  SimulatorModelClass z1Sim;
};

} // namespace SITL
