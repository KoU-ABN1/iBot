#pragma once

#include "common.h"
#include "motor.h"

class TwoDofArm
{
public:
    void pointToTarget(const Eigen::Vector3f &target);

private:
    std::unique_ptr<Motor> left_arm_joint_1 = std::make_unique<Motor>(Motor(handles.left_arm_joint_1));
    std::unique_ptr<Motor> left_arm_joint_2 = std::make_unique<Motor>(Motor(handles.left_arm_joint_2));
    std::unique_ptr<Motor> right_arm_joint_1 = std::make_unique<Motor>(Motor(handles.right_arm_joint_1));
    std::unique_ptr<Motor> right_arm_joint_2 = std::make_unique<Motor>(Motor(handles.right_arm_joint_2));
};