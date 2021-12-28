#include "motor.h"

void Motor::setTargetVelocity(const float vel, const float acc)
{
}

void Motor::setVelocity(const float vel_set)
{
    float v;

    float dv = vel_set - vel_last;
    float dt = data.time_cur - time_last;

    if (dv / dt > ACC_MAX)
        v = vel_last + ACC_MAX * dt;
    else if (dv / dt < -ACC_MAX)
        v = vel_last - ACC_MAX * dt;
    else
        v = vel_set;

    if (v > VEL_MAX)
        v = VEL_MAX;
    else if (v < -VEL_MAX)
        v = -VEL_MAX;

    time_last = data.time_cur;
    vel_last = v;

    simSetJointTargetVelocity(handle, v);
}