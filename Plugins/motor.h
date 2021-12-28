#pragma once

#include "common.h"

class Motor
{
public:
    Motor(int t1) : handle(t1) {}

    void setTargetVelocity(const float velocity, const float acceleration = ACC_DEFAULT);

    void setTargetPosition(const float position,
                           const float upper_velocity = UPPER_VELOCITY_DEFAULT,
                           const float kp = KP_DEFAULT,
                           const float ki = KI_DEFAULT,
                           const float kd = KD_DEFAULT);

private:
    int handle;

    constexpr static float ACC_DEFAULT = -1;
    constexpr static float UPPER_VELOCITY_DEFAULT = 10;
    constexpr static float KP_DEFAULT = 0.1;
    constexpr static float KI_DEFAULT = 0.01;
    constexpr static float KD_DEFAULT = 0;

    float vel_last = 0;
    float time_last = data.time_cur;
};
