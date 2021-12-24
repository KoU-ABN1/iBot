#include "three_part_body.h"

void ThreePartBody::trackCustomerFace()
{
    const float kp = 5;
    const float ki = 0.01;
    const float kd = 0;
    const float tolerance = 0.02;
    const float vel_max = 1;

    static float integral_x = 0;
    static float integral_y = 0;
    static float error_x_last = 0;
    static float error_y_last = 0;

    float error_x = head.x - 0.5;
    float error_y = head.y - 0.5;

    float v1 = 0, v2 = 0;

    if (head.x >= 0 && head.y >= 0)
    {
        if (std::abs(error_x) > tolerance)
            v1 = kp * error_x + ki * integral_x + kd * (error_x - error_x_last);
        else
            integral_x = 0;
        if (std::abs(error_y) > tolerance)
            v2 = kp * error_y + ki * integral_y + kd * (error_y - error_y_last);
        else
            integral_y = 0;
    }

    integral_x += error_x;
    integral_y += error_y;
    error_x_last = error_x;
    error_y_last = error_y;

    std::cout << "body      " << v1 << "    " << v2 << std::endl;

    waist_joint->setVelocity(-v1);
    head_joint_2->setVelocity(-v2);
}