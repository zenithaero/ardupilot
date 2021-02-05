/**
 * @brief Lookup table based simulation
 * @date Created April 30, 2020
 * @author Bertrand Bevillard <bertrand@zenithaero.com>
 */

#pragma once

#include <SITL/SIM_Aircraft.h>
#include <SITL/SIM_ICEngine.h>
#include <Filter/LowPassFilter.h>
#include <vector>
#include <Zenith/Utils/Interp.h>

namespace SITL {


class Startup
{
    public:
        Startup() {
        }
};

class FM {
public:
    Vector3f force;
    Vector3f moment;

    FM() {};

    FM(Vector3f force, Vector3f moment) : force(force), moment(moment) {};

    FM operator+ (FM other) {
        return FM(force + other.force, moment + other.moment);
    }

    FM operator- (FM other) {
        return FM(force - other.force, moment - other.moment);
    }

    FM operator* (float scalar) {
        return FM(force * scalar, moment * scalar);
    }

    void operator+= (FM other) {
        force += other.force;
        moment += other.moment;
    }

    void operator-= (FM other) {
        force -= other.force;
        moment -= other.moment;
    }
};

/*
  a very simple plane simulator
 */
class ZenithSim : public Aircraft {
public:
    ZenithSim(const char *frame_str);

    ~ZenithSim();

    virtual void set_zenith_opts(zenith_sim_opts_t _opts) override;

    /* update model by one time step */
    virtual void update(const struct sitl_input &input) override;

    /* static object creator */
    static Aircraft *create(const char *frame_str) {
        return new ZenithSim(frame_str);
    }

protected:
    zenith_sim_opts_t opts;
    Interp<double> *aeroData;
    const float hover_throttle = 0.7f;
    const float air_density = 1.225; // kg/m^3 at sea level, ISA conditions
    float angle_of_attack;
    float beta;
    Matrix3f I, I_inv;
    std::vector<std::vector<float>> servo_map;
    std::vector<std::vector<float>> motor_map;

    float thrust_scale;

    // Temp
    uint64_t trim_t0 = 0;
    float trim_dt = 0.f;
    float acc = 0;

    // float liftCoeff(float alpha) const;
    // float dragCoeff(float alpha) const;
    FM getAeroFM(const std::vector<double> lookup);
    FM getActuatorFM(const std::vector<double> lookup, float thrLeft, float thrRight, float ail, float elev, float rud);
    FM getDampingFM(const std::vector<double> lookup, Vector3f pqr);
    Vector3f getForce(float inputAileron, float inputElevator, float inputRudder) const;
    Vector3f getTorque(float inputAileron, float inputElevator, float inputRudder, float inputThrust, const Vector3f &force) const;
    void calculate_forces(const struct sitl_input &input, Vector3f &rot_accel);
};

} // namespace SITL