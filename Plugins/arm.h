#pragma once

#include "common.h"
#include "motor.h"

class Arm
{
public:
    Arm(std::string t1) : handle(t1) {}

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

    bool pointToTargetPosition(const Eigen::Vector3f &target_abs, const float &velocity = 1);

private:
    const float ARM_LENGTH = 0.3;

    std::string handle;

    std::unique_ptr<Motor> left_arm_joint_1 = std::make_unique<Motor>(Motor(handles.left_arm_joint_1));
    std::unique_ptr<Motor> left_arm_joint_2 = std::make_unique<Motor>(Motor(handles.left_arm_joint_2));
    std::unique_ptr<Motor> right_arm_joint_1 = std::make_unique<Motor>(Motor(handles.right_arm_joint_1));
    std::unique_ptr<Motor> right_arm_joint_2 = std::make_unique<Motor>(Motor(handles.right_arm_joint_2));

    inline Eigen::Vector3f worldToRobot(Eigen::Vector3f point)
    {
        float x = point[0] * cos(data.robot_yaw) + point[1] * sin(data.robot_yaw);
        float y = -point[0] * sin(data.robot_yaw) + point[1] * cos(data.robot_yaw);

        return Eigen::Vector3f(x, y, point[2]);
    }
};