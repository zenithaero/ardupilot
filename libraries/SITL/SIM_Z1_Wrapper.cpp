/**
 * @brief Z1 wrapper class
 * @date Created Feb 12, 2020
 * @author Bertrand Bevillard <bertrand@zenithaero.com>
 */

#include "SIM_Z1_Wrapper.h"

#include <stdio.h>
#include <errno.h>

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL &hal;

namespace SITL
{

Z1_Wrapper::Z1_Wrapper(const char *frame_str) : Aircraft(frame_str),
                                                last_timestamp(0)
{
    fprintf(stdout, "Starting SITL Z1_Wrapper\n");
    z1Sim.initialize();
}

/*
  decode and send servos
*/
void Z1_Wrapper::send_servos(const struct sitl_input &input)
{
    CmdBus cmdBus = z1Sim.Z1_Sim_DW.CmdBus_e;
    double value[16];
    for (unsigned i = 0; i < 16; ++i)
    {
        // Assumes that the servos are perfectly trimmed. Make sure to use settings reflecting that fact in the simulation
        // TODO: send controller output directly. Then remove the pre-scaling in the model
        value[i] = (input.servos[i] - 1000.0) / 1000.0;
    }

    // Mapping
    cmdBus.thr = value[2];
    cmdBus.ail = value[0];
    cmdBus.elev = value[1];
    cmdBus.rud = value[3];
}

/*
  receive an update from the FDM
  This is a blocking function
 */
void Z1_Wrapper::recv_fdm()
{
    ACBus acBus = z1Sim.Z1_Sim_DW.ACBus_o;

    // Retrieve time
    const double deltat = acBus.time - last_timestamp; // in seconds
    if (deltat < 0)
    { // don't use old data
        time_now_us += 1;
        return;
    }

    // Retreive imu data
    accel_body = Vector3f(acBus.Ab[0],
                          acBus.Ab[1],
                          acBus.Ab[2]);

    gyro = Vector3f(acBus.Wb[0],
                    acBus.Wb[1],
                    acBus.Wb[2]);

    // compute dcm from imu orientation
    Quaternion quat(acBus.quat[0],
                    acBus.quat[1],
                    acBus.quat[2],
                    acBus.quat[3]);
    quat.rotation_matrix(dcm);

    velocity_ef = Vector3f(acBus.Ve[0],
                           acBus.Ve[1],
                           acBus.Ve[2]);

    position = Vector3f(acBus.Xe[0],
                        acBus.Xe[1],
                        acBus.Xe[2]);

    // auto-adjust to simulation frame rate
    time_now_us += static_cast<uint64_t>(deltat * 1.0e6);

    if (deltat < 0.01 && deltat > 0)
    {
        adjust_frame_time(static_cast<float>(1.0 / deltat));
    }
    last_timestamp = acBus.time;
    printf("received packet; pos: %f %f %f\n", position.x, position.y, position.z);
}

/*
  update the Z1 simulation by one time step
 */
void Z1_Wrapper::update(const struct sitl_input &input)
{
    // Update Z1 sim
    send_servos(input);
    z1Sim.step();
    recv_fdm();

    update_position();

    time_advance();
    // update magnetic field
    update_mag_field_bf();
}

} // namespace SITL
