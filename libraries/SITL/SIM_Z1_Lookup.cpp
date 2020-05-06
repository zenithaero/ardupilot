/**
 * @brief Lookup table based simulation
 * @date Created April 30, 2020
 * @author Bertrand Bevillard <bertrand@zenithaero.com>
 */

#include "SIM_Z1_Lookup.h"
#include <Zenith/ZenithAeroData.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <vector>
#include <stdio.h>

using namespace SITL;

Startup startup;
#define INTERP(name, lookup) ({                                     \
    FM _fm;                                                         \
    _fm.force.x = aeroData->get(ZenithAeroData::CX##name, lookup);  \
    _fm.force.y = aeroData->get(ZenithAeroData::CY##name, lookup);  \
    _fm.force.z = -aeroData->get(ZenithAeroData::CL##name, lookup); \
    _fm.moment.x = aeroData->get(ZenithAeroData::Cl##name, lookup); \
    _fm.moment.y = aeroData->get(ZenithAeroData::Cm##name, lookup); \
    _fm.moment.z = aeroData->get(ZenithAeroData::Cn##name, lookup); \
    _fm;                                                            \
})


Z1_Lookup::Z1_Lookup(const char *frame_str) :
    Aircraft(frame_str)
{
    mass = coefficient.mass;
    frame_height = 0.1f;
    num_motors = 1;
    ground_behavior = GROUND_BEHAVIOR_FWD_ONLY;

    // Compute inverse intertia
    bool success = coefficient.I.inverse(coefficient.I_inv);
    if (!success)
        printf("Warning: Inertia matrice inversion failure\n");

    // Create interp struct
    size_t dim;
    dim = sizeof(ZenithAeroData::As) / sizeof(double);
    std::vector<double> as(ZenithAeroData::As, ZenithAeroData::As + dim);
    dim = sizeof(ZenithAeroData::Bs) / sizeof(double);
    std::vector<double> bs(ZenithAeroData::Bs, ZenithAeroData::Bs + dim);
    dim = sizeof(ZenithAeroData::Vs) / sizeof(double);
    std::vector<double> vs(ZenithAeroData::Vs, ZenithAeroData::Vs + dim);
    std::vector<const std::vector<double>> interp = {as, bs, vs};
    aeroData = new Interp(interp);
}

Z1_Lookup::~Z1_Lookup() {
    delete(aeroData);
}

FM Z1_Lookup::getAeroFM(const std::vector<double> lookup) {
    return INTERP(0, lookup);
}

FM Z1_Lookup::getActuatorFM(const std::vector<double> lookup, float ail, float elev, float rud, float thr) {
    FM fm;
    // Actuators
    fm += INTERP(dF_1, lookup) * ail;
    fm += INTERP(dF_2, lookup) * elev;
    fm += INTERP(dF_3, lookup) * rud;
    // Motors
    fm += INTERP(dP_1, lookup) * thr;
    return fm;
}

FM Z1_Lookup::getDampingFM(const std::vector<double> lookup, Vector3f pqr) {
    FM fm;
    float denom = 2 * airspeed;
    if (!is_zero(denom)) {
        fm += INTERP(p, lookup) * pqr.x * coefficient.b * denom;
        fm += INTERP(q, lookup) * pqr.y * coefficient.c * denom;
        fm += INTERP(r, lookup) * pqr.z * coefficient.b * denom;
    }
    return fm;
}


void Z1_Lookup::calculate_forces(const struct sitl_input &input, Vector3f &rot_accel, Vector3f &body_accel)
{
    float ail  = filtered_servo_angle(input, 0);
    float elev = filtered_servo_angle(input, 1);
    float rud   = filtered_servo_angle(input, 3);
    float thr = filtered_servo_range(input, 2);

    // simulate engine RPM
    rpm[0] = thr * 7000;

    // calculate angle of attack
    angle_of_attack = atan2f(velocity_air_bf.z, velocity_air_bf.x);
    beta = atan2f(velocity_air_bf.y,velocity_air_bf.x);
    
    // Build lookup vector
    std::vector<double> lookup = {angle_of_attack, beta, airspeed};

    // Create rotation transformation (clamped to lookup limits)
    std::vector<double> lookupClamped = aeroData->clamp(lookup);
    Matrix3<float> S2B, B2S;
    S2B.from_euler(0.f, lookupClamped[0], 0.f);
    B2S = S2B.transposed();

    // Extract normalization coefficients
    const float s = coefficient.s;
    const float c = coefficient.c;
    const float b = coefficient.b;

    // Rotate gyro vector in stability frame
    Vector3f gyro_sf = B2S * gyro;

    // Calculate dynamic pressure
    float rho = air_density;
    double qbar = 1.0 / 2.0 * rho * pow(airspeed, 2) * s;

    // Get aero FM
    printf("lookup (alpha %.2f, beta %.2f, airspeed %.2f)\n", angle_of_attack, beta, airspeed);
    FM aeroFM = getAeroFM(lookup);
    FM actuatorFM = getActuatorFM(lookup, ail, elev, rud, thr);
    FM dampingFM = getDampingFM(lookup, gyro_sf);
    FM fm = aeroFM + actuatorFM + dampingFM;

    printf("values F(%.2f %.2f %.2f) M(%.2f %.2f %.2f)\n", fm.force.x, fm.force.y, fm.force.z, fm.moment.x, fm.moment.y, fm.moment.z);

    // Move the FMs back to body frame
    fm.force = S2B * fm.force;
    fm.moment = S2B * fm.moment;

    // Denormalize forces & moments
    fm.force *= s * qbar;
    fm.moment.x *= b * s * qbar;
    fm.moment.y *= c * s * qbar;
    fm.moment.z *= b * s * qbar;
    const Vector3f &CGOffset = coefficient.CGOffset;
    Vector3f armMoment = fm.force % CGOffset;
    fm.moment += armMoment;
    
    // Add thrust at low speeds to compensate for the lack of model there
    // Reduce contribution linearly up to minAirspeed
    std::vector<double> zero = {0, 0, 0};
    std::vector<double> zeroClamped = aeroData->clamp(zero);
    double minAirspeed = MAX(1, zeroClamped[2]);
    double r = MAX(0, MIN(1, (minAirspeed - airspeed) / minAirspeed));
    (void)r;
    double thrust = 2 * mass * GRAVITY_MSS * thr;

    // Determine body acceleration & rotational acceleration
    accel_body = (Vector3f(thrust, 0, 0) + fm.force) / mass;
    accel_body /= mass;
    printf("thr %.2f accel_x %.2f airspeed %.2f\n", thr, accel_body.x, airspeed);
    // printf("thr %f, thrust: %f accel_body: %f, %f, %f\n", thr, thrust, accel_body.x, accel_body.y, accel_body.z);

    // Determine rotational accel
    rot_accel = coefficient.I_inv * fm.moment;

    // add some ground friction
    if (on_ground()) {
        Vector3f vel_body = dcm.transposed() * velocity_ef;
        accel_body.x -= vel_body.x * 0.05f;
    }
}
    
/*
  update the plane simulation by one time step
 */
void Z1_Lookup::update(const struct sitl_input &input)
{
    Vector3f rot_accel;

    update_wind(input);
    
    calculate_forces(input, rot_accel, accel_body);
    
    update_dynamics(rot_accel);
    update_external_payload(input);

    // update lat/lon/altitude
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

    // printf("Pos %f %f %f; vel %f %f %f; accel_body %f %f %f; euler %f %f %f\n", position.x, position.y, position.z, velocity_ef.x, velocity_ef.y, velocity_ef.z, accel_body.x, accel_body.y, accel_body.z, r, p, y);
    // Pos 0.000000 0.000000 -0.099976; vel 0.000000 0.000000 0.000000; accel_body 0.000000 0.000000 -9.806650; euler 0.000000 0.000000 -0.122173
}
