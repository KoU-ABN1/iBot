#pragma once

#include "common.h"
#include "motor.h"

class ThreePartBody
{
public:
    void trackCustomerFace();

private:
    std::unique_ptr<Motor> waist_joint = std::make_unique<Motor>(Motor(handles.waist_joint));
    std::unique_ptr<Motor> head_joint_1 = std::make_unique<Motor>(Motor(handles.head_joint_1));
    std::unique_ptr<Motor> head_joint_2 = std::make_unique<Motor>(Motor(handles.head_joint_2));

    inline float limitVelocity(float vel_set, float vel_max);
};