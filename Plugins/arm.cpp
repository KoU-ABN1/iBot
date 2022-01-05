#include "arm.h"

bool Arm::pointToTargetPosition(const Eigen::Vector3f &target_abs, const float &velocity)
{

    if (handle == "left_arm")
    {
        float pos[3];
        simGetObjectPosition(handles.left_arm_joint_1, -1, pos);
        Eigen::Vector3f left_arm_base(pos[0], pos[1], pos[2]);
        Eigen::Vector3f target = target_abs - left_arm_base;
        target = worldToRobot(target);
        std::vector<std::vector<float>> theta = inverseKinematics(target);

        float theta1_cur, theta2_cur;
        simGetJointPosition(handles.left_arm_joint_1, &theta1_cur);
        simGetJointPosition(handles.left_arm_joint_2, &theta2_cur);

        float min_sum = 999;
        bool result_found = false;
        float theta1, theta2;

        for (int i = 0; i < theta.size(); i++)
        {
            if (theta[i][1] > 0)
            {
                float sum = abs(rectifyAngle(-theta1_cur + theta[i][0])) + abs(rectifyAngle(-theta2_cur + theta[i][1]));
                if (sum < min_sum)
                {
                    min_sum = sum;
                    theta1 = theta[i][0];
                    theta2 = theta[i][1];
                    result_found = true;
                }
            }
            //std::cout << "theta " << theta[i][0] << "       " << theta[i][1] << std::endl;
        }

        if (result_found)
        {

            left_arm_joint_1->setTargetPosition(-theta1, velocity);
            left_arm_joint_2->setTargetPosition(theta2, velocity);
        }

        float tolerance = 5 / 180.0 * PI;
        // if (result_found && abs(theta1_cur - theta1) < tolerance && abs(theta2_cur - theta2) < tolerance)
        // {
        //     return true;
        // }
    }

    else if (handle == "right_arm")
    {
        float pos[3];
        simGetObjectPosition(handles.right_arm_joint_1, -1, pos);
        Eigen::Vector3f right_arm_base(pos[0], pos[1], pos[2]);
        Eigen::Vector3f target = target_abs - right_arm_base;
        target = worldToRobot(target);
        std::vector<std::vector<float>> theta = inverseKinematics(target);

        float theta1_cur, theta2_cur;
        simGetJointPosition(handles.right_arm_joint_1, &theta1_cur);
        simGetJointPosition(handles.right_arm_joint_2, &theta2_cur);

        float min_sum = 999;
        bool result_found = false;
        float theta1, theta2;

        for (int i = 0; i < theta.size(); i++)
        {
            if (theta[i][1] > 0)
            {
                float sum = abs(rectifyAngle(-theta1_cur + theta[i][0])) + abs(rectifyAngle(-theta2_cur + theta[i][1]));
                if (sum < min_sum)
                {
                    min_sum = sum;
                    theta1 = theta[i][0];
                    theta2 = theta[i][1];
                    result_found = true;
                }
            }
        }

        if (result_found)
        {
            right_arm_joint_1->setTargetPosition(-theta1, velocity);
            right_arm_joint_2->setTargetPosition(theta2, velocity);
        }
    }

    return false;
}
std::vector<std::vector<float>> Arm::inverseKinematics(const Eigen::Vector3f &target)
{
    std::vector<std::vector<float>> result;
    std::vector<float> theta(2);

    theta[0] = atan2(target[0], target[2]);
    theta[1] = atan2(-target[1] * cos(theta[0]), target[2]);
    if (forwardKinematics(theta).dot(target) > 0)
        result.push_back(theta);

    theta[1] = theta[1] < 0 ? theta[1] + PI : theta[1] - PI;
    if (forwardKinematics(theta).dot(target) > 0)
        result.push_back(theta);

    theta[0] = theta[0] < 0 ? theta[0] + PI : theta[0] - PI;
    theta[1] = atan2(-target[1] * cos(theta[0]), target[2]);
    if (forwardKinematics(theta).dot(target) > 0)
        result.push_back(theta);

    theta[1] = theta[1] < 0 ? theta[1] + PI : theta[1] - PI;
    if (forwardKinematics(theta).dot(target) > 0)
        result.push_back(theta);

    return result;
}

Eigen::Vector3f Arm::forwardKinematics(const std::vector<float> &theta)
{
    float m = -sin(theta[0]) * cos(theta[1]) * ARM_LENGTH;
    float n = sin(theta[1]) * ARM_LENGTH;
    float p = -cos(theta[0]) * cos(theta[1]) * ARM_LENGTH;

    return Eigen::Vector3f(m, n, p);
}
