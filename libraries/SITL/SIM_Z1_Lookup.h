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
        // printf("$ find index: %f, found %lu frac %f\n", value, k, frac);
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
    Interp(const std::vector<const std::vector<double>> &lookups, oorBehavior_e oorBehavior = INTERPOLATE) 
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
            // printf("- contribution %lu\n", k);
            double frac = 1;
            for (size_t l = 0; l < lookups.size(); l++) {
                size_t on = (k >> l) & 1;
                indList[l] = indices[l].idx + on;
                frac *= on ? indices[l].frac : 1 - indices[l].frac;
                // printf(" # lookup %lu, on %lu, index %lu, frac %f\n", l, on, indList[l], on ? indices[l].frac : 1 - indices[l].frac);
            }
            // Now get value at indList
            value += frac * getValue(table, indList);
        }
        return value;
    }

    static void test() {
        // double table[4*3*2];
        // for (size_t k = 0; k < 4 * 3 * 2; k++)
        //     table[k] = k;

        // std::vector<double> l0 = {0, 1, 2, 3};
        // std::vector<double> l1 = {0, 1, 2};
        // std::vector<double> l2 = {0, 1};
        // std::vector<const std::vector<double>> lookups = {l0, l1, l2};

        // Interp interp(lookups);

        // for (size_t i = 0; i < 4; i++)
        //     for (size_t j = 0; j < 3; j++)
        //         for (size_t k = 0; k < 2; k++) {
        //             std::vector<size_t> indices = {i, j, k};
        //             double val = interp.getValue(table, indices);
        //             size_t idx = i * 2 * 3 + j * 2 + k;
        //             printf("Expected value %.0f actual %.0f\n", table[idx], val);
        //         }
        
        // double table[] = {1, 2};
        // std::vector<double> l0 = {0, 1};
        // std::vector<const std::vector<double>> lookups = {l0};
        // Interp interp(lookups);
        // for (double k = -2.0 - 0.3; k < 2.5; k += 0.5) {
        //     std::vector<double> val = {k};
        //     double v = interp.get(table, val);
        //     printf("k: %f; value: %f\n", k, v);
        // }

        // std::vector<double> x1 = {1.0000, 1.5000, 2.0000, 2.5000, 3.0000};
        // std::vector<double> x2 = {1.0000, 1.5000, 2.0000};
        // std::vector<double> x3 = {4.0000, 4.5000, 5.0000};
        // double table[] = {0.5129, 0.4319,  0.7859,  0.7092,  0.3608,  0.1582,  0.1922,  0.8044,  0.9404,  0.4605,  0.4460,  0.4107,  0.1160,  0.3365,  0.6012,  0.4714,  0.0111,  0.4156,  0.3504,  0.5083,  0.1194,  0.0781,  0.1733,  0.1176,  0.1449,  0.2331,  0.2720,  0.0950,  0.5281,  0.6344,  0.3693,  0.0861,  0.6261,  0.7178,  0.9339,  0.9280,  0.4337,  0.5729,  0.8624,  0.0336,  0.3933,  0.8351,  0.6617,  0.2268,  0.9213};
        // double xq1[] = {1.0000, 1.0000, 3.0000, 1.2000, 2.2000, 1.7000};
        // double xq2[] = {1.0000, 2.0000, 2.0000, 1.6000, 1.1000, 1.1000};
        // double xq3[] = {4.0000, 4.0000, 5.0000, 4.1000, 4.6000, 4.8000};

        // std::vector<const std::vector<double>> lookups = {x1, x2, x3};
        // Interp interp(lookups);

        // for (size_t k = 0; k < sizeof(xq1) / sizeof(*xq1); k++) {
        //     std::vector<double> val = {xq1[k], xq2[k], xq3[k]};
        //     double v = interp.get(table, val);
        //     printf("k: %lu; value: %f\n", k, v);
        // }

        // printf("test done!\n");
        // exit(-1);
    }
};

class Startup
{
    public:
        Startup() {
            Interp::test();
        }
};

/*
  a very simple plane simulator
 */
class Z1_Lookup : public Aircraft {
public:
    Z1_Lookup(const char *frame_str);

    /* update model by one time step */
    virtual void update(const struct sitl_input &input) override;

    /* static object creator */
    static Aircraft *create(const char *frame_str) {
        return new Z1_Lookup(frame_str);
    }

protected:
    const float hover_throttle = 0.7f;
    const float air_density = 1.225; // kg/m^3 at sea level, ISA conditions
    float angle_of_attack;
    float beta;

    struct {
        // from last_letter skywalker_2013/aerodynamics.yaml
        // thanks to Georacer!
        float s = 0.45;
        float b = 1.88;
        float c = 0.24;
        float c_lift_0 = 0.56;
        float c_lift_deltae = 0;
        float c_lift_a = 6.9;
        float c_lift_q = 0;
        float mcoeff = 50;
        float oswald = 0.9;
        float alpha_stall = 0.4712;
        float c_drag_q = 0;
        float c_drag_deltae = 0.0;
        float c_drag_p = 0.1;
        float c_y_0 = 0;
        float c_y_b = -0.98;
        float c_y_p = 0;
        float c_y_r = 0;
        float c_y_deltaa = 0;
        float c_y_deltar = -0.2;
        float c_l_0 = 0;
        float c_l_p = -1.0;
        float c_l_b = -0.12;
        float c_l_r = 0.14;
        float c_l_deltaa = 0.25;
        float c_l_deltar = -0.037;
        float c_m_0 = 0.045;
        float c_m_a = -0.7;
        float c_m_q = -20;
        float c_m_deltae = 1.0;
        float c_n_0 = 0;
        float c_n_b = 0.25;
        float c_n_p = 0.022;
        float c_n_r = -1;
        float c_n_deltaa = 0.00;
        float c_n_deltar = 0.1;
        float deltaa_max = 0.3491;
        float deltae_max = 0.3491;
        float deltar_max = 0.3491;
        // the X CoG offset should be -0.02, but that makes the plane too tail heavy
        // in manual flight. Adjusted to -0.15 gives reasonable flight
        Vector3f CGOffset{-0.15, 0, -0.05};
    } coefficient;

    float thrust_scale;

    float liftCoeff(float alpha) const;
    float dragCoeff(float alpha) const;
    Vector3f getForce(float inputAileron, float inputElevator, float inputRudder) const;
    Vector3f getTorque(float inputAileron, float inputElevator, float inputRudder, float inputThrust, const Vector3f &force) const;
    void calculate_forces(const struct sitl_input &input, Vector3f &rot_accel, Vector3f &body_accel);
};

} // namespace SITL
