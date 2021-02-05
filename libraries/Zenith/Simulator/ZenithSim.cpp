/**
 * @brief Lookup table based simulation
 * @date Created April 30, 2020
 * @author Bertrand Bevillard <bertrand@zenithaero.com>
 */

#include <Zenith/Simulator/ZenithSim.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <vector>
#include <stdio.h>
#include <Zenith/constants.h>

using namespace SITL;

Startup startup;
#define INTERP(name, lookup, vec_length, vec_idx) ({                                                                  \
    FM _fm;                                                                                                             \
    _fm.force.x = aeroData->get(ModelAeroData::CX##name, DIM(ModelAeroData::CX##name), lookup, vec_length, vec_idx);  \
    _fm.force.y = aeroData->get(ModelAeroData::CY##name, DIM(ModelAeroData::CX##name), lookup, vec_length, vec_idx);  \
    _fm.force.z = aeroData->get(ModelAeroData::CZ##name, DIM(ModelAeroData::CX##name), lookup, vec_length, vec_idx);  \
    _fm.moment.x = aeroData->get(ModelAeroData::Cl##name, DIM(ModelAeroData::CX##name), lookup, vec_length, vec_idx); \
    _fm.moment.y = aeroData->get(ModelAeroData::Cm##name, DIM(ModelAeroData::CX##name), lookup, vec_length, vec_idx); \
    _fm.moment.z = aeroData->get(ModelAeroData::Cn##name, DIM(ModelAeroData::CX##name), lookup, vec_length, vec_idx); \
    _fm;                                                                                                                \
})

ZenithSim::ZenithSim(const char *frame_str) :
    Aircraft(frame_str)
{
    mass = ModelConfig::M;
    frame_height = 0.1f;
    num_motors = 2;
    ground_behavior = GROUND_BEHAVIOR_FWD_ONLY;
    build_mat(ModelConfig::servoMap, servo_map);
    build_mat(ModelConfig::motorMap, motor_map);

    // Compute inverse intertia
    I = Matrix3f(
        ModelConfig::I[0][0], ModelConfig::I[0][1], ModelConfig::I[0][2],
        ModelConfig::I[1][0], ModelConfig::I[1][1], ModelConfig::I[1][2],
        ModelConfig::I[2][0], ModelConfig::I[2][1], ModelConfig::I[2][2]
    );
    bool success = I.inverse(I_inv);
    if (!success) {
        fprintf(stderr, "Warning: Inertia matrice inversion failure\n");
        // exit(-1);
    }

    // Create interp struct
    std::vector<double> as(ModelAeroData::As, ModelAeroData::As + DIM(ModelAeroData::As));
    std::vector<double> bs(ModelAeroData::Bs, ModelAeroData::Bs + DIM(ModelAeroData::Bs));
    std::vector<double> vs(ModelAeroData::Vs, ModelAeroData::Vs + DIM(ModelAeroData::Vs));
    std::vector<std::vector<double>> interp = {as, bs, vs};
    aeroData = new Interp<double>(interp);

    // // TEST Lookup
    // double _alpha[] = {-2, 3, 5, 6};
    // double _beta[] = {-2, -4, 3, 4};
    // double _airspeed[] = {11, 15, 18, 22};
    // for (size_t k = 0; k < sizeof(_alpha) / sizeof(double); k++) {
    //     std::vector<double> lookup = {_alpha[k], _beta[k], _airspeed[k]};
    //     double CX0 = aeroData->get(ModelAeroData::CX0, lookup);
    //     double Cmq = aeroData->get(ModelAeroData::Cmq, lookup);
    //     double CmdF_1 = aeroData->get(ModelAeroData::CmdF, lookup, 0);
    //     double CmdF_2 = aeroData->get(ModelAeroData::CmdF, lookup, 1);
    //     double CmdF_3 = aeroData->get(ModelAeroData::CmdF, lookup, 2);
    //     printf("CX0 %.4f Cmq %.4f CmdF [%.4f, %.4f, %.4f]\n", CX0, Cmq, CmdF_1, CmdF_2, CmdF_3);
    // }
    // exit(-1);
    // // MATLAB // alpha = [-2, 3, 5, 6];
    // // MATLAB // beta = [-2, -4, 3, 4];
    // // MATLAB // airspeed = [11, 15, 18, 22];
    // // MATLAB // for k = 1:length(alpha)
    // // MATLAB //     CX0 = getDataPoints('xuav', 'CX0', alpha(k), beta(k), airspeed(k));
    // // MATLAB //     Cmq = getDataPoints('xuav', 'Cmq', alpha(k), beta(k), airspeed(k));
    // // MATLAB //     CmdF = getDataPoints('xuav', 'CmdF', alpha(k), beta(k), airspeed(k));
    // // MATLAB //     fprintf('CX0 %.4f Cmq %.4f CldF [%.4f, %.4f, %.4f]\n', CX0, Cmq, CmdF(1), CmdF(2), CmdF(3)); 
    // // MATLAB // end
}

ZenithSim::~ZenithSim() {
    delete(aeroData);
}

void ZenithSim::set_zenith_opts(zenith_sim_opts_t _opts) {
    opts = _opts;
}

FM ZenithSim::getAeroFM(const std::vector<double> lookup) {
    // auto fm = INTERP(0, lookup);
    // printf("lookup: [%.4f, %.4f, %.4f]: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]\n", lookup[0], lookup[1], lookup[2], fm.force.x, fm.force.y, fm.force.z, fm.moment.x, fm.moment.y, fm.moment.z);
    FM fm = INTERP(0, lookup, 1, 0);
    // if (trim_dt > 15)
        // printf("aero: [%.4f, %.4f, %.4f]: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]\n", lookup[0], lookup[1], lookup[2], fm.force.x, fm.force.y, fm.force.z, fm.moment.x, fm.moment.y, fm.moment.z);
    return fm;
}

FM ZenithSim::getActuatorFM(const std::vector<double> lookup, float thrLeft, float thrRight, float ail, float elev, float rud) {
    FM actuator;
    // Actuator map
    std::vector<float> vec = {ail, elev, rud, 0, 0};
    std::vector<float> act = matmul(servo_map, vec);
    for (size_t k = 0; k < act.size(); k++)
        actuator += INTERP(dF, lookup, act.size(), k) * act[k] * ModelConfig::servoMaxDeg[0][k];
    // if (trim_dt > 15)
    //     printf("actuator: [%.4f, %.4f, %.4f]: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]\n", lookup[0], lookup[1], lookup[2], actuator.force.x, actuator.force.y, actuator.force.z, actuator.moment.x, actuator.moment.y, actuator.moment.z);

    // Motor map
    vec = {thrLeft, thrRight};
    act = matmul(motor_map, vec);
    FM motor;
    for (size_t k = 0; k < act.size(); k++)
        motor += INTERP(dP, lookup, act.size(), k) * act[k] * ModelConfig::motorMax[0][k];
    
    
    // if (trim_dt > 15)
    //     printf("motors: [%.4f, %.4f, %.4f]: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]\n", lookup[0], lookup[1], lookup[2], motor.force.x, motor.force.y, motor.force.z, motor.moment.x, motor.moment.y, motor.moment.z);
    
    FM sum = actuator + motor;
    // if (trim_dt > 15)
        // printf("act_sum: [%.4f, %.4f, %.4f]: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]\n", lookup[0], lookup[1], lookup[2], sum.force.x, sum.force.y, sum.force.z, sum.moment.x, sum.moment.y, sum.moment.z);

    return actuator + motor;
}

FM ZenithSim::getDampingFM(const std::vector<double> lookup, Vector3f pqr) {
    FM fm;
    float denom = 2 * airspeed;
    if (!is_zero(denom)) {
        fm += INTERP(p, lookup, 1, 0) * pqr.x * ModelConfig::b * (1 / denom);
        fm += INTERP(q, lookup, 1, 0) * pqr.y * ModelConfig::c * (1 / denom);
        fm += INTERP(r, lookup, 1, 0) * pqr.z * ModelConfig::b * (1 / denom);
    }
    return fm;
}

float dynamic_pressure(float rho, float airspeed) {
    return 1.0 / 2.0 * rho * pow(airspeed, 2);
}

void denormalize(FM &fm, float qbar) {
    const float s = ModelConfig::S;
    const float c = ModelConfig::c;
    const float b = ModelConfig::b;
    fm.force *= s * qbar;
    fm.moment.x *= b * s * qbar;
    fm.moment.y *= c * s * qbar;
    fm.moment.z *= b * s * qbar;
} 


void ZenithSim::calculate_forces(const struct sitl_input &input, Vector3f &rot_accel)
{
    float thrLeft = filtered_servo_range(input, 0);
    float thrRight = filtered_servo_range(input, 1);
    float ail = filtered_servo_angle(input, 2);
    float elev = filtered_servo_angle(input, 3);
    float rud  = filtered_servo_angle(input, 4);
    // printf("ELEV (SIM) %.2f\n", elev);

    // Configure trim state if needed
    if (trim_t0 == 0) {
        trim_t0 = time_now_us;
    }
    trim_dt = (time_now_us - trim_t0) / 1e6f;


    size_t trim_idx = 1; // TODO: pass as a parameter
    if (opts.open_loop) {
        thrLeft = ControllerData::trim.thrLeft[trim_idx] / ModelConfig::motorMax[0][0];
        thrRight = ControllerData::trim.thrRight[trim_idx]  / ModelConfig::motorMax[0][1];
        ail = ControllerData::trim.ailDeg[trim_idx] / ModelConfig::servoMaxDeg[0][0];
        elev = ControllerData::trim.elevDeg[trim_idx] / ModelConfig::servoMaxDeg[0][1];
        rud = ControllerData::trim.rudDeg[trim_idx] / ModelConfig::servoMaxDeg[0][2];
    }

    if (opts.trim) {
        if (trim_dt < 15) {
            gyro = Vector3f(0, 0, 0);
            dcm.from_euler(0, 0, 0);
            velocity_ef = Vector3f(0, 0, 0);
            velocity_air_bf = Vector3f(0, 0, 0);
            velocity_air_ef = Vector3f(0, 0, 0);
            airspeed_pitot = 0.f;
            airspeed = 0.f;
        } else if (trim_dt > 15 && trim_dt < 55) {
            gyro = Vector3f(0, 0, 0);
            dcm.from_euler(
                ControllerData::trim.phi[trim_idx],
                ControllerData::trim.theta[trim_idx],
                ControllerData::trim.psi[trim_idx]
            );
            Vector3f velocity_bf = {
                ControllerData::trim.ux[trim_idx],
                ControllerData::trim.uy[trim_idx],
                ControllerData::trim.uz[trim_idx],
            };
            velocity_ef = dcm * velocity_bf;
            velocity_air_bf = velocity_bf;
            velocity_air_ef = velocity_ef;
            airspeed_pitot = constrain_float(velocity_air_bf * Vector3f(1.0f, 0.0f, 0.0f), 0.0f, 120.0f);
            airspeed = velocity_air_ef.length();
            // printf("airspeed: %.2f\n", airspeed);
            // lookup: [0.0000, 0.0000, 12.0000]: [0.0008, 0.0000, 0.0197, -0.0000, 0.0305, -0.0000]

            // Fix altitude
            position.z = -100;
        } else if (trim_dt > 75 && opts.auto_stop) {
            // Terminate the simulation
            exit(-1);
        }
        if (trim_dt > 50) {
            // elev += 0.1 / ModelConfig::servoMaxDeg[0][1];
            // ail += 0.1 / ModelConfig::servoMaxDeg[0][0];
        }
    }
    
    if (opts.longitudinal) {
        gyro.x = gyro.z = 0.f;
        velocity_ef.y = 0.f;
        velocity_air_bf.y = 0.f;
        velocity_air_ef.y = 0.f;
        float r, p, y;
        dcm.to_euler(&r, &p, &y);
        dcm.from_euler(0.f, p, 0.f);
    }
    if (opts.lateral) {
        gyro.y = 0.f;
        velocity_ef.x = velocity_ef.z = 0.f;
        velocity_air_bf.x = velocity_air_bf.z = 0.f;
        velocity_air_ef.x = velocity_air_ef.z = 0.f;
        float r, p, y;
        dcm.to_euler(&r, &p, &y);
        dcm.from_euler(r, 0.f, y);
    }

    // printf("thr [%.2f, %.2f], ail %.2f, elev %.2f, rud %.2f\n",
    //     thrLeft,
    //     thrRight,
    //     ail * ModelConfig::servoMaxDeg[0][0],
    //     elev * ModelConfig::servoMaxDeg[0][1],
    //     rud * ModelConfig::servoMaxDeg[0][2]
    // );

    // Log actuator states
    actuators[0] = thrLeft;
    actuators[1] = thrRight;
    actuators[2] = ail * ModelConfig::servoMaxDeg[0][0];
    actuators[3] = elev * ModelConfig::servoMaxDeg[0][1];
    actuators[4] = rud * ModelConfig::servoMaxDeg[0][2];

    // simulate engine RPM
    rpm[0] = thrLeft * 7000;
    rpm[1] = thrRight * 7000;

    // calculate angle of attack
    angle_of_attack = atan2f(velocity_air_bf.z, velocity_air_bf.x);
    beta = atan2f(velocity_air_bf.y, velocity_air_bf.x);
    
    // Build lookup vector
    std::vector<double> lookup = {angle_of_attack * RAD_TO_DEG, beta * RAD_TO_DEG, airspeed};

    // Create rotation transformation (clamped to lookup limits)
    std::vector<double> lookupClamped = aeroData->clamp(lookup);

    // Calculate dynamic pressure
    double qbar = dynamic_pressure(air_density, airspeed);

    // Get aero FM
    // printf("lookup (alphaDeg %.2f, betaDeg %.2f, airspeed %.2f)\n", lookup[0], lookup[1], lookup[2]);
    FM aeroFM = getAeroFM(lookup);
    FM actuatorFM = getActuatorFM(lookup, thrLeft, thrRight, ail, elev, rud);
    FM dampingFM = getDampingFM(lookup, gyro);
    // printf("qbar: %.2f\n", qbar);
    // printf("aeroFM (%.3f, %.3f, %.3f; %.3f, %.3f, %.3f)\n", aeroFM.force.x, aeroFM.force.y, aeroFM.force.z, aeroFM.moment.x, aeroFM.moment.y, aeroFM.moment.z);
    // printf("actuatorFM (%.3f, %.3f, %.3f; %.3f, %.3f, %.3f)\n", actuatorFM.force.x, actuatorFM.force.y, actuatorFM.force.z, actuatorFM.moment.x, actuatorFM.moment.y, actuatorFM.moment.z);
    // printf("dampingFM (%.3f, %.3f, %.3f; %.3f, %.3f, %.3f)\n", dampingFM.force.x, dampingFM.force.y, dampingFM.force.z, dampingFM.moment.x, dampingFM.moment.y, dampingFM.moment.z);
    FM fm = aeroFM + actuatorFM + dampingFM;
    // printf("fm0: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]\n", fm.force.x, fm.force.y, fm.force.z, fm.moment.x, fm.moment.y, fm.moment.z);

    // (0.03, 0.00, -0.54; 0.00, -0.00, -0.54)

    // printf("values F(%.2f %.2f %.2f) M(%.2f %.2f %.2f)\n", fm.force.x, fm.force.y, fm.force.z, fm.moment.x, fm.moment.y, fm.moment.z);
    // Denormalize forces & moments
    denormalize(fm, qbar);
    const Vector3f CGOffset(ModelConfig::CG[0][0], ModelConfig::CG[0][1], ModelConfig::CG[0][2]);
    Vector3f armMoment = fm.force % CGOffset;
    fm.moment += armMoment;

    // Add low speed thrust model (for takeoff)
    std::vector<float> vec = {thrLeft, thrRight};
    std::vector<float> act = matmul(motor_map, vec);
    std::vector<double> lookup_0 = { lookup[0], lookup[1], 0.f };
    lookup_0 = aeroData->clamp(lookup_0);
    float qbar_0 = dynamic_pressure(air_density, lookup_0[2]);
    FM fm_0;
    for (size_t k = 0; k < act.size(); k++)
        fm_0 += INTERP(dP, lookup, act.size(), k) * act[k] * ModelConfig::motorMax[0][k];
    denormalize(fm_0, CLAMP(qbar_0 - qbar, 0, qbar_0));
    fm += fm_0;

    
    // AVL static thrust motor model
    // double coeff = ModelConfig::thrustStatic - qbar * ModelConfig::thrustGain;
    // double thrust = CLAMP(GRAVITY_MSS * thr * coeff, 0, INFINITY);
    // fm.force += Vector3f(thrust, 0, 0);
    // printf("Thrust force: %.2f\n", thrust);

    // ASWING thrust boost (low velocity)
    // double thrust = ModelConfig::thrustStatic * 100 * (13 - airspeed ) / 13 * thr;
    // thrust = CLAMP(thrust, 0, INFINITY);
    // fm.force += Vector3f(thrust, 0, 0);

    // Try adding a bit of drag
    // auto drag = -velocity_air_bf * 0.3f;
    // fm.force += drag;

    // Store forces & moments
    force_bf = fm.force;
    moment_bf = fm.moment;

    // printf("force_bf: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]\n", force_bf.x, force_bf.y, force_bf.z, moment_bf.x, moment_bf.y, moment_bf.z);


    // Determine body acceleration & rotational acceleration
    accel_body = fm.force / mass;
    rot_accel = I_inv * fm.moment;

    // printf("accel_bf: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]\n", accel_body.x, accel_body.y, accel_body.z, rot_accel.x, rot_accel.y, rot_accel.z);

    // force_bf: [0.0963, 0.0000, -24.8710, -0.0001, -0.3700, -0.0000]
    // accel_bf: [0.0351, 0.0000, -9.0770, -0.0004, -2.9294, -0.0002]

    // TEMP Debug
    // printf("force: %.4f, %.4f, %.4f; moment: %.4f, %.4f, %.4f\n", force_bf.x, force_bf.y, force_bf.z, moment_bf.x, moment_bf.y, moment_bf.z);
    // printf("accel: %.4f, %.4f, %.4f; rot_accel: %.4f, %.4f, %.4f\n", accel_body.x, accel_body.y, accel_body.z, rot_accel.x, rot_accel.y, rot_accel.z);

    // add some ground friction
    if (on_ground()) {
        Vector3f vel_body = dcm.transposed() * velocity_ef;
        accel_body.x -= vel_body.x * 0.05f;
    }
}
    
/*
  update the plane simulation by one time step
 */
void ZenithSim::update(const struct sitl_input &input)
{
    Vector3f rot_accel;
    update_wind(input);

    
    calculate_forces(input, rot_accel);
    update_dynamics(rot_accel);

    update_external_payload(input);
    update_position();
    time_advance();

    // update magnetic field
    update_mag_field_bf();

    float r, p, y;
    dcm.to_euler(&r, &p, &y);

    // Reset precentage if needed
    
    uint8_t remaining = AP::battery().capacity_remaining_pct(AP_BATT_PRIMARY_INSTANCE);
    if (remaining < 80) {
        AP::battery().reset_remaining(1 << AP_BATT_PRIMARY_INSTANCE, 81.0f);
    }

    // Clip velocity TODO: messes with estimation (would require a recomputation of the acceleration)
    // printf("velocity: %.2f\n", velocity_ef.length());
    // float V = velocity_ef.length();
    // if (V > FLT_EPSILON)
    //     velocity_ef = velocity_ef.normalized() * MIN(25, V);


    // printf("Pos %f %f %f; vel %f %f %f; accel_body %f %f %f; euler %f %f %f\n", position.x, position.y, position.z, velocity_ef.x, velocity_ef.y, velocity_ef.z, accel_body.x, accel_body.y, accel_body.z, r, p, y);
    // Pos 0.000000 0.000000 -0.099976; vel 0.000000 0.000000 0.000000; accel_body 0.000000 0.000000 -9.806650; euler 0.000000 0.000000 -0.122173
}
