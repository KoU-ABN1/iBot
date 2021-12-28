#pragma once

#include "common.h"

class Motor
{
public:
    Motor(int temp) : handle(temp) {}

    void setTargetVelocity(const float vel, const float acc);

    void setVelocity(const float vel_set);

    void setPosition(const float pos_set);

    void setVelMax(const float vel_max)
    {
        VEL_MAX = vel_max;
    }

    void setAccMax(const float acc_max)
    {
        ACC_MAX = acc_max;
    }

private:
    int handle;

    float VEL_MAX = 10;
    float ACC_MAX = 2;

    float vel_last = 0;
    float time_last = data.time_cur;
};
