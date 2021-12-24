#include "motor.h"

void Motor::setVelocity(const float vel_set)
{
    float v;

    float dv = vel_set - vel_last;
    float dt = time_cur - time_last;

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

    time_last = time_cur;
    vel_last = v;

    //std::cout << vel_set << "          " << v << std::endl;

    simSetJointTargetVelocity(handle, v);
}