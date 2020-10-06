/**
 * @brief Lookup table based simulation
 * @date Created April 30, 2020
 * @author Bertrand Bevillard <bertrand@zenithaero.com>
 */

#include "SIM_Z1_Lookup.h"
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <vector>
#include <stdio.h>
#include <Zenith/constants.h>

using namespace SITL;

Startup startup;
#define INTERP(name, lookup, tableIdx) ({                                                                  \
    FM _fm;                                                                                                \
    _fm.force.x = aeroData->get(ModelAeroData::CX##name, DIM(ModelAeroData::CX##name), lookup, tableIdx);  \
    _fm.force.y = aeroData->get(ModelAeroData::CY##name, DIM(ModelAeroData::CX##name), lookup, tableIdx);  \
    _fm.force.z = aeroData->get(ModelAeroData::CZ##name, DIM(ModelAeroData::CX##name), lookup, tableIdx);  \
    _fm.moment.x = aeroData->get(ModelAeroData::Cl##name, DIM(ModelAeroData::CX##name), lookup, tableIdx); \
    _fm.moment.y = aeroData->get(ModelAeroData::Cm##name, DIM(ModelAeroData::CX##name), lookup, tableIdx); \
    _fm.moment.z = aeroData->get(ModelAeroData::Cn##name, DIM(ModelAeroData::CX##name), lookup, tableIdx); \
    _fm;                                                                                                   \
})


Z1_Lookup::Z1_Lookup(const char *frame_str) :
    Aircraft(frame_str)
{
    mass = ModelConfig::M;
    frame_height = 0.1f;
    num_motors = 1;
    ground_behavior = GROUND_BEHAVIOR_FWD_ONLY;
    build_mat(ModelConfig::servoMap, servo_map);

    // Compute inverse intertia
    I = Matrix3f(
        ModelConfig::I[0][0], ModelConfig::I[0][1], ModelConfig::I[0][2],
        ModelConfig::I[1][0], ModelConfig::I[1][1], ModelConfig::I[1][2],
        ModelConfig::I[2][0], ModelConfig::I[2][1], ModelConfig::I[2][2]
    );
    bool success = I.inverse(I_inv);
    if (!success) {
        fprintf(stderr, "Warning: Inertia matrice inversion failure\n");
        exit(-1);
    }

    // Create interp struct
    std::vector<double> as(ModelAeroData::As, ModelAeroData::As + DIM(ModelAeroData::As));
    std::vector<double> bs(ModelAeroData::Bs, ModelAeroData::Bs + DIM(ModelAeroData::Bs));
    std::vector<double> vs(ModelAeroData::Vs, ModelAeroData::Vs + DIM(ModelAeroData::Vs));
    std::vector<const std::vector<double>> interp = {as, bs, vs};
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

Z1_Lookup::~Z1_Lookup() {
    delete(aeroData);
}

FM Z1_Lookup::getAeroFM(const std::vector<double> lookup) {
    // auto fm = INTERP(0, lookup);
    // printf("lookup: [%.4f, %.4f, %.4f]: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]\n", lookup[0], lookup[1], lookup[2], fm.force.x, fm.force.y, fm.force.z, fm.moment.x, fm.moment.y, fm.moment.z);
    return INTERP(0, lookup, 0);
}

FM Z1_Lookup::getActuatorFM(const std::vector<double> lookup, float ail, float elev, float rud) {
    FM fm;
    // Apply map
    std::vector<float> vec = {ail, elev, rud, 0, 0};
    std::vector<float> act = matmul(servo_map, vec);
    // Lookup map
    for (size_t k = 0; k < act.size(); k++) {
        fm += INTERP(dF, lookup, k) * act[k] * ModelConfig::servoMaxDeg[0][k];
    }
    return fm;
}

FM Z1_Lookup::getDampingFM(const std::vector<double> lookup, Vector3f pqr) {
    FM fm;
    float denom = 2 * airspeed;
    if (!is_zero(denom)) {
        fm += INTERP(p, lookup, 0) * pqr.x * ModelConfig::b * (1 / denom);
        fm += INTERP(q, lookup, 0) * pqr.y * ModelConfig::c * (1 / denom);
        fm += INTERP(r, lookup, 0) * pqr.z * ModelConfig::b * (1 / denom);
    }
    return fm;
}


void Z1_Lookup::calculate_forces(const struct sitl_input &input, Vector3f &rot_accel)
{
    float ail  = filtered_servo_angle(input, 0);
    float elev = filtered_servo_angle(input, 1);
    float rud  = filtered_servo_angle(input, 3);
    float thr = filtered_servo_range(input, 2);

    printf("ail %.2f, elev %.2f, rud: %.2f, thr: %.2f\n", ail * 25, elev * 15, rud * 15, thr);

    // Dummy state
    // gyro = Vector3f(3.5f, -4.f, -2.8f);
    // velocity_air_bf = Vector3f(14.f, 2.f, -4.f);
    // dcm.from_euler(0.4, -0.6, 0.3);
    // velocity_air_ef = dcm * velocity_air_bf;
    // airspeed = velocity_air_ef.length();
    // ail = 8.f;
    // elev = -12.f;
    // rud = 10.f;
    // thr = 0.5f;

    // TEMP
    thr = 0.02124;
    ail = 0;
    rud = 0;
    elev = 0.86159 / 15;

    if (t0 == 0)
        t0 = time_now_us;
    float dt = (time_now_us - t0) / 1e6f;
    if (dt < 15) {
        gyro = Vector3f(0, 0, 0);
        dcm.from_euler(0.00000, -0.02408, 0.00000);
        velocity_ef = Vector3f(0, 0, 0);
        velocity_air_bf = Vector3f(0, 0, 0);
        velocity_air_ef = Vector3f(0, 0, 0);
        airspeed_pitot = 0.f;
        airspeed = 0.f;
    }
    if (dt > 15 && dt < 45) {
        gyro = Vector3f(0, 0, 0);
        dcm.from_euler(0.00000, -0.02408, 0.00000);
        Vector3f velocity_bf = {14.99565, -0.00000, -0.36111};
        velocity_ef = dcm * velocity_bf;
        position.z = -200;

        // update_position();
        // AIS computations
        velocity_air_bf = velocity_bf;
        velocity_air_ef = velocity_ef;
        airspeed_pitot = constrain_float(velocity_air_bf * Vector3f(1.0f, 0.0f, 0.0f), 0.0f, 120.0f);
        airspeed = velocity_air_ef.length();
    }
    if (dt > 45)
        elev += 1.f / 15;
    if (dt > 65)
        exit(-1);

    // Force longitudinal
    // float roll, pitch, yaw;
    // dcm.to_euler(&roll, &pitch, &yaw);
    // dcm.from_euler(0, pitch, 0);

    // if (dt > 90)
    //     exit(-1);

    // else {
    //     ail += 1;
    //     elev -= 1;
    // }
    // if (dt > 30)
    //     exit(-1);
    // elev += 1;
    // TEMP END

    actuators[0] = ail;
    actuators[1] = elev;
    actuators[2] = rud;
    actuators[3] = thr;


    // simulate engine RPM
    rpm[0] = thr * 7000;

    // calculate angle of attack
    angle_of_attack = atan2f(velocity_air_bf.z, velocity_air_bf.x);
    beta = atan2f(velocity_air_bf.y, velocity_air_bf.x);
    
    // Build lookup vector
    std::vector<double> lookup = {angle_of_attack * RAD_TO_DEG, beta * RAD_TO_DEG, airspeed};

    // Create rotation transformation (clamped to lookup limits)
    std::vector<double> lookupClamped = aeroData->clamp(lookup);

    // Extract normalization coefficients
    const float s = ModelConfig::S;
    const float c = ModelConfig::c;
    const float b = ModelConfig::b;

    // Calculate dynamic pressure
    float rho = air_density;
    double qbar = 1.0 / 2.0 * rho * pow(airspeed, 2);

    // Get aero FM
    // printf("lookup (alpha %.2f, beta %.2f, airspeed %.2f)\n", angle_of_attack, beta, airspeed);
    FM aeroFM = getAeroFM(lookup);
    FM actuatorFM = getActuatorFM(lookup, ail, elev, rud);
    FM dampingFM = getDampingFM(lookup, gyro);
    // printf("qbar: %.2f\n", qbar);
    // printf("aeroFM (%.3f, %.3f, %.3f; %.3f, %.3f, %.3f)\n", aeroFM.force.x, aeroFM.force.y, aeroFM.force.z, aeroFM.moment.x, aeroFM.moment.y, aeroFM.moment.z);
    // printf("actuatorFM (%.3f, %.3f, %.3f; %.3f, %.3f, %.3f)\n", actuatorFM.force.x, actuatorFM.force.y, actuatorFM.force.z, actuatorFM.moment.x, actuatorFM.moment.y, actuatorFM.moment.z);
    // printf("dampingFM (%.3f, %.3f, %.3f; %.3f, %.3f, %.3f)\n", dampingFM.force.x, dampingFM.force.y, dampingFM.force.z, dampingFM.moment.x, dampingFM.moment.y, dampingFM.moment.z);
    FM fm = aeroFM + actuatorFM + dampingFM;

    // (0.03, 0.00, -0.54; 0.00, -0.00, -0.54)

    // printf("values F(%.2f %.2f %.2f) M(%.2f %.2f %.2f)\n", fm.force.x, fm.force.y, fm.force.z, fm.moment.x, fm.moment.y, fm.moment.z);

    // Denormalize forces & moments
    fm.force *= s * qbar;
    fm.moment.x *= b * s * qbar;
    fm.moment.y *= c * s * qbar;
    fm.moment.z *= b * s * qbar;
    const Vector3f CGOffset(ModelConfig::CG[0][0], ModelConfig::CG[0][1], ModelConfig::CG[0][2]);
    Vector3f armMoment = fm.force % CGOffset;
    fm.moment += armMoment;
    
    // Simple static thrust motor model
    double coeff = ModelConfig::thrustStatic - qbar * ModelConfig::thrustGain;
    double thrust = CLAMP(GRAVITY_MSS * thr * coeff, 0, INFINITY);
    
    // thrust *= MIN(1, MAX(0, 0.85 - 0.01 * airspeed));
    fm.force += Vector3f(thrust, 0, 0);
    // printf("Thrust force: %.2f\n", thrust);

    // Try adding a bit of drag
    // auto drag = -velocity_air_bf * 0.3f;
    // fm.force += drag;

    // Store forces & moments
    force_bf = fm.force;
    moment_bf = fm.moment;

    // Determine body acceleration & rotational acceleration
    accel_body = fm.force / mass;
    rot_accel = I_inv * fm.moment;

    // TEMP Debug
    // printf("force: %.4f, %.4f, %.4f; moment: %.4f, %.4f, %.4f\n", force_bf.x, force_bf.y, force_bf.z, moment_bf.x, moment_bf.y, moment_bf.z);
    // printf("accel: %.4f, %.4f, %.4f; rot_accel: %.4f, %.4f, %.4f\n", accel_body.x, accel_body.y, accel_body.z, rot_accel.x, rot_accel.y, rot_accel.z);

    // add some ground friction
    if (on_ground()) {
        Vector3f vel_body = dcm.transposed() * velocity_ef;
        accel_body.x -= vel_body.x * 0.3f;
    }
}
    
/*
  update the plane simulation by one time step
 */
void Z1_Lookup::update(const struct sitl_input &input)
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
