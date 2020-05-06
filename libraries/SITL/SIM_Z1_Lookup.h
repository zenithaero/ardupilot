/**
 * @brief Lookup table based simulation
 * @date Created April 30, 2020
 * @author Bertrand Bevillard <bertrand@zenithaero.com>
 */

#pragma once

#include "SIM_Aircraft.h"
#include "SIM_ICEngine.h"
#include <Filter/LowPassFilter.h>
#include <vector>

namespace SITL {

class Interp {
    
typedef struct {
    const size_t idx;
    const double frac;
} lookupIdx_t;

typedef enum {
    CLIP,
    INTERPOLATE
} oorBehavior_e;

private:
    const std::vector<const std::vector<double>> lookups;
    oorBehavior_e oorBehavior;
    // std::vector<size_t> lookupLength;

    lookupIdx_t findIndex(size_t lookupIdx, double value) const {
        auto lookup = lookups[lookupIdx];
        size_t k;
        for (k = 0; (k < lookup.size() - 2) && (value > lookup[k + 1]); k++);
        double frac = (value - lookup[k]) / (lookup[k + 1] - lookup[k]);
        if (oorBehavior == CLIP)
            frac = MAX(0.0, MIN(1.0, frac));
        return (lookupIdx_t) {
            .idx = k,
            .frac = frac
        };
    }

    double getValue(const double *table, const std::vector<size_t> &indices) const {
        assert(indices.size() > 0);
        size_t idx = indices[0];
        for (size_t l = 1; l < lookups.size(); l++) {
            idx = idx * lookups[l].size() + indices[l];
        };
        return table[idx];
    }

public:
    Interp(const std::vector<const std::vector<double>> &lookups, oorBehavior_e oorBehavior = CLIP) 
        : lookups(lookups), oorBehavior(oorBehavior) {
            assert(lookups.size() > 0 && lookups.size() < 32);
            for (auto lookup : lookups) {
                // Lookup must be large enough for interpolation
                assert(lookup.size() > 1);
                // Assert that the lookup is monotically increasing
                for (size_t k = 0; k < lookup.size() - 1; k++)
                    assert(lookup[k] < lookup[k + 1]);
            }
        };

    std::vector<double> clamp(const std::vector<double> &vector) const {
        std::vector<double> rtn;
        for (size_t k = 0; k < lookups.size(); k++) {
            double min = lookups[k][0];
            double max = lookups[k][lookups.size() - 1];
            rtn.push_back(MAX(min, MIN(max, vector[k])));
        }
        return rtn;
    }
    
    double get(const double *table, const std::vector<double> &values) const {
        assert(values.size() == lookups.size());
        std::vector<lookupIdx_t> indices;
        std::vector<size_t> indList;
        for (size_t k = 0; k < lookups.size(); k++) {
            auto val = values[k];
            indices.push_back(findIndex(k, val));
            indList.push_back(0);
        }
        // Now loop through all possible indices n-uplets
        double value = 0.0;
        // printf("preparing lookup. nPoints: %d\n", 1 << lookups.size());
        for (size_t k = 0; k < 1 << lookups.size(); k++) {
            double frac = 1;
            for (size_t l = 0; l < lookups.size(); l++) {
                size_t on = (k >> l) & 1;
                indList[l] = indices[l].idx + on;
                frac *= on ? indices[l].frac : 1 - indices[l].frac;
            }
            // Now get value at indList
            value += frac * getValue(table, indList);
        }
        return value;
    }
};

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
class Z1_Lookup : public Aircraft {
public:
    Z1_Lookup(const char *frame_str);

    ~Z1_Lookup();

    /* update model by one time step */
    virtual void update(const struct sitl_input &input) override;

    /* static object creator */
    static Aircraft *create(const char *frame_str) {
        return new Z1_Lookup(frame_str);
    }

protected:
    Interp *aeroData;
    const float hover_throttle = 0.7f;
    const float air_density = 1.225; // kg/m^3 at sea level, ISA conditions
    float angle_of_attack;
    float beta;

    struct {
        float mass = 2.6;
        float s = 0.5350;
        float b = 1.9000;
        float c = 0.2920;
        Vector3f CGOffset{0, 0, 0};
        Matrix3f I{
            0.2343f, -0.1821e-08f, 0.1173e-02f,
            -0.1821e-08f, 0.1425f, -0.1232e-09f,
            0.1173e-02f, -0.1232E-09f, 0.3731f
        };
        Matrix3f I_inv;
    } coefficient;

    float thrust_scale;

    // float liftCoeff(float alpha) const;
    // float dragCoeff(float alpha) const;
    FM getAeroFM(const std::vector<double> lookup);
    FM getActuatorFM(const std::vector<double> lookup, float ail, float elev, float rud, float thr);
    FM getDampingFM(const std::vector<double> lookup, Vector3f pqr);
    Vector3f getForce(float inputAileron, float inputElevator, float inputRudder) const;
    Vector3f getTorque(float inputAileron, float inputElevator, float inputRudder, float inputThrust, const Vector3f &force) const;
    void calculate_forces(const struct sitl_input &input, Vector3f &rot_accel, Vector3f &body_accel);
};

} // namespace SITL
