#include "motor.h"

void Motor::setTargetVelocity(const float vel, const float acc)
{
    simSetObjectInt32Parameter(handle, sim_jointintparam_ctrl_enabled, 0); // enable velocity control mode;

    if (acc < 0)
    {
        simSetJointTargetVelocity(handle, vel);
    }
    else
    {
        float v;
        float dv = vel - vel_last;
        float dt = data.time_cur - time_last;

        if (dv / dt > acc)
            v = vel_last + acc * dt;
        else if (dv / dt < -acc)
            v = vel_last - acc * dt;
        else
            v = vel;

        time_last = data.time_cur;
        vel_last = v;

        simSetJointTargetVelocity(handle, v);
    }
}

void Motor::setTargetPosition(const float pos)
{
    simSetObjectInt32Parameter(handle, sim_jointintparam_ctrl_enabled, 1); // enable position control mode
}