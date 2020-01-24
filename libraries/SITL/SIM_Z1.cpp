/**
 * @brief Z1 Simulator class
 * @date Created Jan 17, 2020
 * @author Bertrand Bevillard <bertrand@zenithaero.com>
 */

#include "SIM_Z1.h"

#include <stdio.h>

using namespace SITL;

Z1::Z1(const char *frame_str) : Aircraft(frame_str)
{
    mass = 2.0f;

    /*
       scaling from motor power to Newtons. Allows the plane to hold
       vertically against gravity when the motor is at hover_throttle
    */
    thrust_scale = (mass * GRAVITY_MSS) / hover_throttle;
    frame_height = 0.1f;
    num_motors = 1;

    ground_behavior = GROUND_BEHAVIOR_FWD_ONLY;

    if (strstr(frame_str, "-dspoilers"))
    {
        dspoilers = true;
    }
    if (strstr(frame_str, "-launch"))
    {
        have_launcher = true;
        launch_speed = 13;
        launch_time = 3;
    }
}

/*
  the following functions are from last_letter
  https://github.com/Georacer/last_letter/blob/master/last_letter/src/aerodynamicsLib.cpp
  many thanks to Georacer!
 */
float Z1::liftCoeff() const
{
    const float alpha0 = coefficient.alpha_stall;
    const float M = coefficient.mcoeff;
    const float c_lift_0 = coefficient.c_lift_0;
    const float c_lift_a0 = coefficient.c_lift_a;

    // clamp the value of alpha to avoid exp(90) in calculation of sigmoid
    float alpha_clamped = alpha;
    const float max_alpha_delta = 0.8f;
    if (alpha - alpha0 > max_alpha_delta)
    {
        alpha_clamped = alpha0 + max_alpha_delta;
    }
    else if (alpha0 - alpha > max_alpha_delta)
    {
        alpha_clamped = alpha0 - max_alpha_delta;
    }
    double sigmoid = (1 + exp(-M * (alpha_clamped - alpha0)) + exp(M * (alpha_clamped + alpha0))) / (1 + exp(-M * (alpha_clamped - alpha0))) / (1 + exp(M * (alpha_clamped + alpha0)));
    double linear = (1.0 - sigmoid) * (c_lift_0 + c_lift_a0 * alpha_clamped);                                        //Lift at small AoA
    double flatPlate = sigmoid * (2 * copysign(1, alpha_clamped) * pow(sin(alpha_clamped), 2) * cos(alpha_clamped)); //Lift beyond stall

    float result = linear + flatPlate;
    return result;
}

float Z1::dragCoeff() const
{
    const float b = coefficient.b;
    const float s = coefficient.s;
    const float c_drag_p = coefficient.c_drag_p;
    const float c_lift_0 = coefficient.c_lift_0;
    const float c_lift_a0 = coefficient.c_lift_a;
    const float oswald = coefficient.oswald;

    double AR = pow(b, 2) / s;
    double c_drag_a = c_drag_p + pow(c_lift_0 + c_lift_a0 * alpha, 2) / (M_PI * oswald * AR);

    return c_drag_a;
}

// Torque calculation function
Vector3f Z1::getTorque(float inputAileron, float inputElevator, float inputRudder, float inputThrust, const Vector3f &force) const
{
    //calculate aerodynamic torque
    float effective_airspeed = airspeed;

    const float s = coefficient.s;
    const float c = coefficient.c;
    const float b = coefficient.b;
    const float c_l_0 = coefficient.c_l_0;
    const float c_l_b = coefficient.c_l_b;
    const float c_l_p = coefficient.c_l_p;
    const float c_l_r = coefficient.c_l_r;
    const float c_l_deltaa = coefficient.c_l_deltaa;
    const float c_l_deltar = coefficient.c_l_deltar;
    const float c_m_0 = coefficient.c_m_0;
    const float c_m_a = coefficient.c_m_a;
    const float c_m_q = coefficient.c_m_q;
    const float c_m_deltae = coefficient.c_m_deltae;
    const float c_n_0 = coefficient.c_n_0;
    const float c_n_b = coefficient.c_n_b;
    const float c_n_p = coefficient.c_n_p;
    const float c_n_r = coefficient.c_n_r;
    const float c_n_deltaa = coefficient.c_n_deltaa;
    const float c_n_deltar = coefficient.c_n_deltar;
    const Vector3f &CGOffset = coefficient.CGOffset;

    float rho = air_density;

    //read angular rates
    double p = gyro.x;
    double q = gyro.y;
    double r = gyro.z;

    double qbar = 1.0 / 2.0 * rho * pow(effective_airspeed, 2) * s; //Calculate dynamic pressure
    double la, na, ma;
    if (is_zero(effective_airspeed))
    {
        la = 0;
        ma = 0;
        na = 0;
    }
    else
    {
        la = qbar * b * (c_l_0 + c_l_b * beta + c_l_p * b * p / (2 * effective_airspeed) + c_l_r * b * r / (2 * effective_airspeed) + c_l_deltaa * inputAileron + c_l_deltar * inputRudder);
        ma = qbar * c * (c_m_0 + c_m_a * alpha + c_m_q * c * q / (2 * effective_airspeed) + c_m_deltae * inputElevator);
        na = qbar * b * (c_n_0 + c_n_b * beta + c_n_p * b * p / (2 * effective_airspeed) + c_n_r * b * r / (2 * effective_airspeed) + c_n_deltaa * inputAileron + c_n_deltar * inputRudder);
    }

    // Add torque to to force misalignment with CG
    // r x F, where r is the distance from CoG to CoL
    la += CGOffset.y * force.z - CGOffset.z * force.y;
    ma += -CGOffset.x * force.z + CGOffset.z * force.x;
    na += -CGOffset.y * force.x + CGOffset.x * force.y;

    return Vector3f(la, ma, na);
}

// Force calculation function from last_letter
Vector3f Z1::getForce(float inputAileron, float inputElevator, float inputRudder) const
{
    const float c_drag_q = coefficient.c_drag_q;
    const float c_lift_q = coefficient.c_lift_q;
    const float s = coefficient.s;
    const float c = coefficient.c;
    const float b = coefficient.b;
    const float c_drag_deltae = coefficient.c_drag_deltae;
    const float c_lift_deltae = coefficient.c_lift_deltae;
    const float c_y_0 = coefficient.c_y_0;
    const float c_y_b = coefficient.c_y_b;
    const float c_y_p = coefficient.c_y_p;
    const float c_y_r = coefficient.c_y_r;
    const float c_y_deltaa = coefficient.c_y_deltaa;
    const float c_y_deltar = coefficient.c_y_deltar;

    float rho = air_density;

    //request lift and drag alpha-coefficients from the corresponding functions
    double c_lift_a = liftCoeff();
    double c_drag_a = dragCoeff();

    //convert coefficients to the body frame
    double c_x_a = -c_drag_a * cos(alpha) + c_lift_a * sin(alpha);
    double c_x_q = -c_drag_q * cos(alpha) + c_lift_q * sin(alpha);
    double c_z_a = -c_drag_a * sin(alpha) - c_lift_a * cos(alpha);
    double c_z_q = -c_drag_q * sin(alpha) - c_lift_q * cos(alpha);

    //read angular rates
    double p = gyro.x;
    double q = gyro.y;
    double r = gyro.z;

    //calculate aerodynamic force
    double qbar = 1.0 / 2.0 * rho * pow(airspeed, 2) * s; //Calculate dynamic pressure
    double ax, ay, az;
    if (is_zero(airspeed))
    {
        ax = 0;
        ay = 0;
        az = 0;
    }
    else
    {
        ax = qbar * (c_x_a + c_x_q * c * q / (2 * airspeed) - c_drag_deltae * cos(alpha) * fabs(inputElevator) + c_lift_deltae * sin(alpha) * inputElevator);
        // split c_x_deltae to include "abs" term
        ay = qbar * (c_y_0 + c_y_b * beta + c_y_p * b * p / (2 * airspeed) + c_y_r * b * r / (2 * airspeed) + c_y_deltaa * inputAileron + c_y_deltar * inputRudder);
        az = qbar * (c_z_a + c_z_q * c * q / (2 * airspeed) - c_drag_deltae * sin(alpha) * fabs(inputElevator) - c_lift_deltae * cos(alpha) * inputElevator);
        // split c_z_deltae to include "abs" term
    }
    return Vector3f(ax, ay, az);
}

void Z1::calculate_forces(const struct sitl_input &input, Vector3f &rot_accel, Vector3f &body_accel)
{
    uint64_t now = AP_HAL::millis64();
    float aileron = filtered_servo_angle(input, 0);
    float elevator = filtered_servo_angle(input, 1);
    float rudder = filtered_servo_angle(input, 3);
    if (!launch_start_ms && input.servos[6] > 1700)
        launch_start_ms = now;

    // // fake a vtail plane
    // float ch1 = elevator;
    // float ch2 = rudder;
    // // this matches VTAIL_OUTPUT==2
    // elevator = (ch2 - ch1) / 2.0f;
    // rudder = (ch2 + ch1) / 2.0f;

    if (dspoilers)
    {
        // fake a differential spoiler plane. Use outputs 1, 2, 4 and 5
        float dspoiler1_left = filtered_servo_angle(input, 0);
        float dspoiler1_right = filtered_servo_angle(input, 1);
        float dspoiler2_left = filtered_servo_angle(input, 3);
        float dspoiler2_right = filtered_servo_angle(input, 4);
        float elevon_left = (dspoiler1_left + dspoiler2_left) / 2;
        float elevon_right = (dspoiler1_right + dspoiler2_right) / 2;
        aileron = (elevon_right - elevon_left) / 2;
        elevator = (elevon_left + elevon_right) / 2;
        rudder = fabsf(dspoiler1_right - dspoiler2_right) / 2 - fabsf(dspoiler1_left - dspoiler2_left) / 2;
    }
    //printf("Aileron: %.1f elevator: %.1f rudder: %.1f\n", aileron, elevator, rudder);

    float thrust = filtered_servo_range(input, 2);

    // calculate angle of attack
    alpha = atan2f(velocity_air_bf.z, velocity_air_bf.x);
    beta = atan2f(velocity_air_bf.y, velocity_air_bf.x);

    Vector3f force = getForce(aileron, elevator, rudder);
    rot_accel = getTorque(aileron, elevator, rudder, thrust, force);

    if (launch_start_ms > 0 && now - launch_start_ms < launch_time * 1000)
    {
        printf("LAUNCH FORCE\n");
        // Simulate launch setup
        float launch_accel = launch_speed / launch_time;
        force.x += launch_accel;
        force.z += launch_accel / 3; // TODO: check behavior
    }

    // simulate engine RPM
    rpm[0] = thrust * 7000;

    // scale thrust to newtons
    thrust *= thrust_scale;

    accel_body = Vector3f(thrust, 0, 0) + force;
    accel_body /= mass;

    // add some noise
    if (thrust_scale > 0)
    {
        add_noise(fabsf(thrust) / thrust_scale);
    }

    if (on_ground())
    {
        // add some ground friction
        Vector3f vel_body = dcm.transposed() * velocity_ef;
        accel_body.x -= vel_body.x * 0.3f;
    }
}

/*
  update the plane simulation by one time step
 */
void Z1::update(const struct sitl_input &input)
{
    Vector3f rot_accel;

    update_wind(input);

    calculate_forces(input, rot_accel, accel_body);

    // TEMP: rotate slowly around axes
    // float rate = 3.14f / 4;
    // Vector3f slowGyro(0, 0, 0);
    // const float delta_time = frame_time_us * 1.0e-6f;
    // dcm.rotate(slowGyro * delta_time);
    // dcm.normalize();

    update_dynamics(rot_accel);
    update_external_payload(input);

    // update lat/lon/altitude
    update_position();
    time_advance();

    // update magnetic field
    update_mag_field_bf();
}
