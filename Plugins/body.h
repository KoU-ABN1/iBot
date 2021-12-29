#pragma once

#include "common.h"
#include "motor.h"

class Body
{
public:
    void trackCustomerFace(const float vel_base = 0,
                           const float vel_max = 0.5,
                           const float kp = 5,
                           const float ki = 0.1,
                           const float kd = 0);

    void stop();

private:
    std::unique_ptr<Motor> waist_joint = std::make_unique<Motor>(Motor(handles.waist_joint));
    std::unique_ptr<Motor> head_joint_1 = std::make_unique<Motor>(Motor(handles.head_joint_1));
    std::unique_ptr<Motor> head_joint_2 = std::make_unique<Motor>(Motor(handles.head_joint_2));

    inline float limitVelocity(float vel_set, float vel_max);
};