#pragma once

#include "common.h"

class Motor
{
public:
    Motor(int t1) : handle(t1) {}

    void setTargetVelocity(const float vel, const float acc = ACC_DEFAULT);

    void setTargetPosition(const float pos);

private:
    int handle;

    constexpr static float ACC_DEFAULT = -1;

    float vel_last = 0;
    float time_last = data.time_cur;
};
