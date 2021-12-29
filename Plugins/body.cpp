#include "body.h"

void Body::trackCustomerFace(const float vel_base, const float vel_max, const float kp, const float ki, const float kd)
{
    static float integral_x = 0;
    static float integral_y = 0;
    static float error_x_last = 0;
    static float error_y_last = 0;

    float error_x = data.head_x - 0.5;
    float error_y = data.head_y - 0.5;

    float v1 = 0, v2 = 0;
    float tolerance = 0.01;

    if (data.head_x >= 0 && data.head_y >= 0)
    {
        if (std::abs(error_x) > tolerance)
            v1 = vel_base + kp * error_x + ki * integral_x + kd * (error_x - error_x_last);
        else
            integral_x = 0;
        if (std::abs(error_y) > tolerance)
            v2 = vel_base + kp * error_y + ki * integral_y + kd * (error_y - error_y_last);
        else
            integral_y = 0;
    }
    else
    {
        integral_x = 0;
        integral_y = 0;
    }

    integral_x += error_x;
    integral_y += error_y;
    error_x_last = error_x;
    error_y_last = error_y;

    v1 = limitVelocity(v1, vel_max);
    v2 = limitVelocity(v2, vel_max);

    waist_joint->setTargetVelocity(-v1, -1);
    head_joint_2->setTargetVelocity(-v2, -1);
}

void Body::stop()
{
    waist_joint->setTargetVelocity(0);
    head_joint_1->setTargetVelocity(0);
    head_joint_2->setTargetVelocity(0);
}

inline float Body::limitVelocity(float vel_set, float vel_max)
{
    if (vel_set > vel_max)
        vel_set = vel_max;
    else if (vel_set < -vel_max)
        vel_set = -vel_max;
    return vel_set;
}