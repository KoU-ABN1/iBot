#pragma once

#include "common.h"

class Motor
{
public:
    Motor(int handle_) : handle(handle_) {}

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

    float VEL_MAX = 3;
    float ACC_MAX = 1;

    float vel_last = 0;
    float time_last = time_cur;
};
