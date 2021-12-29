#pragma once

#include "common.h"
#include "motor.h"

class Arm
{
public:
    /**
     * @brief Forward Kinematics of the 2-dof manipulator arm
     * 
     * @param theta Joint angles
     * @return Eigen::Vector3f End-effector postion
     */
    Eigen::Vector3f forwardKinematics(const std::vector<float> &theta);

    /**
     * @brief Inverse Kinematics of the 2-dof manipulator arm
     * 
     * @param target Taget position of the end-effector
     * @return std::vector<std::vector<float>> The output joint angles, which are stored in vectors (multiple results may exist)
     */
    std::vector<std::vector<float>> inverseKinematics(const Eigen::Vector3f &target);

private:
    const float ARM_LENGTH = 0.3;

    std::unique_ptr<Motor> left_arm_joint_1 = std::make_unique<Motor>(Motor(handles.left_arm_joint_1));
    std::unique_ptr<Motor> left_arm_joint_2 = std::make_unique<Motor>(Motor(handles.left_arm_joint_2));
    std::unique_ptr<Motor> right_arm_joint_1 = std::make_unique<Motor>(Motor(handles.right_arm_joint_1));
    std::unique_ptr<Motor> right_arm_joint_2 = std::make_unique<Motor>(Motor(handles.right_arm_joint_2));
};