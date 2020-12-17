#pragma once

#include <vector>
#include <assert.h>
#include <stdio.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Common/AP_Common.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Math/AP_Math.h>
#include <Filter/LowPassFilter.h>
#include <Zenith/Utils/Interp.h>
#include <Zenith/constants.h>

class LinearController {
public:
    template<size_t n, size_t n_tas, size_t n_k>
    LinearController(
        AP_AHRS &ahrs,
        const char* (&stateNames)[n],
        const char* (&expectedNames)[n],
        const float (&_K_tas)[n_tas],
        const float (&_K_grid)[n_k]);

    ~LinearController();

    /* Do not allow copies */
    LinearController(const LinearController &other) = delete;
    LinearController &operator=(const LinearController&) = delete;

protected:
	AP_AHRS &ahrs;
    uint64_t t_prev;
    float dt;
    // Gridded gain
    LowPassFilter<float> tas_filter;
    std::vector<float> K_grid;
    Interp<float> *K_interp;
    std::vector<std::vector<float>> K;
    Interp<float> *tas_interp;
    std::vector<float> tas_value = { 0.f };

    void update();
    void init_gains();
};

class DoubletHandler {
public:
    DoubletHandler() {};

    DoubletHandler(SRV_Channel::Aux_servo_function_t _channel, float _deflection, float _duration, float _settle_time)
        : channel(_channel), deflection(_deflection), duration(_duration), settle_time(_settle_time) {};

    void start();
    void clear();
    bool udpate(float last_command);
    
protected:
    // Parameters
    SRV_Channel::Aux_servo_function_t channel;
    float deflection;
    float duration;
    float settle_time;

    // Vars
    uint64_t t_start = 0;
    LowPassFilter<float> command_filter;
};
