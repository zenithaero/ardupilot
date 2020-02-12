/**
 * @brief Z1 Matlab connection class
 * @date Created Feb 8, 2020
 * @author Bertrand Bevillard <bertrand@zenithaero.com>
 */

#include "SIM_Z1_Matlab.h"

#include <stdio.h>
#include <errno.h>

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL &hal;

namespace SITL
{

Z1_Matlab::Z1_Matlab(const char *frame_str) : Aircraft(frame_str),
                                              last_timestamp(0),
                                              socket_sitl{true}
{
    fprintf(stdout, "Starting SITL Z1_Matlab\n");
}

/*
  Create and set in/out socket
*/
void Z1_Matlab::set_interface_ports(const char *address, const int port_in, const int port_out)
{
    // try to bind to a specific port so that if we restart ArduPilot
    // Z1_Matlab keeps sending us packets. Not strictly necessary but
    // useful for debugging
    if (!socket_sitl.bind("0.0.0.0", port_in))
    {
        fprintf(stderr, "SITL: socket in bind failed on sim in : %d  - %s\n", port_in, strerror(errno));
        fprintf(stderr, "Aborting launch...\n");
        exit(1);
    }
    printf("Bind %s:%d for SITL in\n", "127.0.0.1", port_in);
    socket_sitl.reuseaddress();
    socket_sitl.set_blocking(false);

    _Z1_Matlab_address = address;
    _Z1_Matlab_port = port_out;
    printf("Setting Z1_Matlab interface to %s:%d \n", _Z1_Matlab_address, _Z1_Matlab_port);
}

/*
  decode and send servos
*/
void Z1_Matlab::send_servos(const struct sitl_input &input)
{
    servo_packet pkt;
    for (unsigned i = 0; i < 16; ++i)
    {
        // Assumes that the servos are perfectly trimmed. Make sure to use settings reflecting that fact in the simulation
        // TODO: send controller output directly. Then remove the pre-scaling in the model
        pkt.servos[i] = (input.servos[i] - 1000) / 1000.0f;
    }
    socket_sitl.sendto(&pkt, sizeof(pkt), _Z1_Matlab_address, _Z1_Matlab_port);
}

/*
  receive an update from the FDM
  This is a blocking function
 */
void Z1_Matlab::recv_fdm(const struct sitl_input &input)
{
    fdm_packet pkt;

    /*
      we re-send the servo packet every 0.1 seconds until we get a
      reply. This allows us to cope with some packet loss to the FDM
     */
    size_t size = 0;
    while (size != sizeof(pkt))
    {
        size = socket_sitl.recv(&pkt, sizeof(pkt), 100);
        send_servos(input);
        // Reset the timestamp after a long disconnection, also catch Z1_Matlab reset
        if (get_wall_time_us() > last_wall_time_us + Z1_Matlab_TIMEOUT_US)
        {
            last_timestamp = 0;
        }
    }

    const double deltat = pkt.timestamp - last_timestamp; // in seconds
    if (deltat < 0)
    { // don't use old packet
        time_now_us += 1;
        return;
    }
    // get imu stuff
    accel_body = Vector3f(pkt.imu_linear_acceleration_xyz[0],
                          pkt.imu_linear_acceleration_xyz[1],
                          pkt.imu_linear_acceleration_xyz[2]);

    gyro = Vector3f(pkt.imu_angular_velocity_rpy[0],
                    pkt.imu_angular_velocity_rpy[1],
                    pkt.imu_angular_velocity_rpy[2]);

    // compute dcm from imu orientation
    Quaternion quat(pkt.imu_orientation_quat[0],
                    pkt.imu_orientation_quat[1],
                    pkt.imu_orientation_quat[2],
                    pkt.imu_orientation_quat[3]);
    quat.rotation_matrix(dcm);

    velocity_ef = Vector3f(pkt.velocity_xyz[0],
                           pkt.velocity_xyz[1],
                           pkt.velocity_xyz[2]);

    position = Vector3f(pkt.position_xyz[0],
                        pkt.position_xyz[1],
                        pkt.position_xyz[2]);

    // auto-adjust to simulation frame rate
    time_now_us += static_cast<uint64_t>(deltat * 1.0e6);

    if (deltat < 0.01 && deltat > 0)
    {
        adjust_frame_time(static_cast<float>(1.0 / deltat));
    }
    last_timestamp = pkt.timestamp;
    // printf("received packet; pos: %f %f %f\n", position.x, position.y, position.z);
}

/*
  Drain remaining data on the socket to prevent phase lag.
 */
void Z1_Matlab::drain_sockets()
{
    const uint16_t buflen = 1024;
    char buf[buflen];
    ssize_t received;
    errno = 0;
    do
    {
        received = socket_sitl.recv(buf, buflen, 0);
        if (received < 0)
        {
            if (errno != EAGAIN && errno != EWOULDBLOCK && errno != 0)
            {
                fprintf(stderr, "error recv on socket in: %s \n",
                        strerror(errno));
            }
        }
        else
        {
            // fprintf(stderr, "received from control socket: %s\n", buf);
        }
    } while (received > 0);
}

/*
  update the Z1_Matlab simulation by one time step
 */
void Z1_Matlab::update(const struct sitl_input &input)
{
    send_servos(input);
    recv_fdm(input);
    update_position();

    time_advance();
    // update magnetic field
    update_mag_field_bf();
    drain_sockets();
}

} // namespace SITL
