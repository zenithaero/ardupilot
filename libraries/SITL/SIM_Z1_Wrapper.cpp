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
    CmdBus *cmdBus = &z1Sim.Simulator_DW.CmdBus_e;
    double value[16];
    for (unsigned i = 0; i < 16; ++i)
    {
        // Assumes that the servos are perfectly trimmed. Make sure to use settings reflecting that fact in the simulation
        // TODO: send controller output directly. Then remove the pre-scaling in the model
        value[i] = (input.servos[i] - 1500.f) / 500.f;
    }

    // Mapping
    cmdBus->thr = value[2];
    cmdBus->ail = value[0];
    cmdBus->elev = value[1];
    cmdBus->rud = value[3];

    // printf("thr: %f; elev %f\n", cmdBus->thr, cmdBus->elev);
}

/*
  receive an update from the FDM
  This is a blocking function
 */
void Z1_Wrapper::recv_fdm()
{
    ACBus acBus = z1Sim.Simulator_DW.ACBus_o;

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

    // Simulate airspeed
    velocity_air_ef = velocity_ef + wind_ef;

    // velocity relative to airmass in body frame
    velocity_air_bf = dcm.transposed() * velocity_air_ef;

    // airspeed
    airspeed = velocity_air_bf.length();

    // airspeed as seen by a fwd pitot tube (limited to 120m/s)
    airspeed_pitot = constrain_float(velocity_air_bf * Vector3f(1.0f, 0.0f, 0.0f), 0.0f, 120.0f);

    // auto-adjust to simulation frame rate
    time_now_us += static_cast<uint64_t>(deltat * 1.0e6);

    // if (deltat < 0.01 && deltat > 0)
    // {
    //     adjust_frame_time(static_cast<float>(1.0 / deltat));
    // }
    last_timestamp = acBus.time;

    float r, p, y;
    dcm.to_euler(&r, &p, &y);
    // printf("Pos %f %f %f; vel %f %f %f; accel_body %f %f %f; euler %f %f %f\n", position.x, position.y, position.z, velocity_ef.x, velocity_ef.y, velocity_ef.z, accel_body.x, accel_body.y, accel_body.z, r, p, y);
    // printf("deltaTime: %f; time %f \n", deltat, (float)time_now_us / 1e6);
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
