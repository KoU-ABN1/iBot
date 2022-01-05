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

// bool Arm::pointToTargetPosition(Eigen::Vector3f target, const float upper_velocity)
// {

//     std::cout << "input pos " << target[0] << "   " << target[1] << "   " << target[2] << std::endl;
//     std::vector<std::vector<float>> theta = inverseKinematics(target);
//     if (name == "left_arm")
//     {
//         //std::cout << min_sum_theta << std::endl;
//         float theta1_cur, theta2_cur;
//         simGetJointPosition(handles.left_arm_joint_1, &theta1_cur);
//         simGetJointPosition(handles.left_arm_joint_2, &theta2_cur);

//         simSetObjectInt32Parameter(handles.left_arm_joint_1, sim_jointintparam_ctrl_enabled, 1); // enable postion control
//         simSetObjectInt32Parameter(handles.left_arm_joint_2, sim_jointintparam_ctrl_enabled, 1);
//         float min_value = 999;
//         float target_theta[2];
//         bool result_found = false;
//         for (int i = 0; i < theta.size(); i++)
//         {

//             if (theta[i][1] < 0)
//             {
//                 float theta1_diff = rectifyAngle(theta[i][0] - theta1_cur);
//                 float theta2_diff = rectifyAngle(theta[i][1] - theta2_cur);

//                 if (abs(theta1_diff) + abs(theta2_diff) < min_value)
//                 {
//                     min_value = abs(theta1_diff) + abs(theta2_diff);
//                     target_theta[0] = theta[i][0];
//                     target_theta[1] = theta[i][1];
//                     result_found = true;
//                 }
//             }
//             Eigen::Vector3f result = forwardKinematics(theta[i]);
//             std::cout << "theta " << theta[i][0] << "       " << theta[i][1] << std::endl;
//             std::cout << "output pos " << result[0] << "      " << result[1] << "      " << result[2] << "      " << std::endl;
//         }
//         std::cout << "min_value " << min_value << std::endl;
//         //std::cout << "target_theta " << abs(rectifyAngle(target_theta[0] - theta1_cur)) << "        " << abs(rectifyAngle(target_theta[1] - theta2_cur)) << std::endl;
//         std::cout << "theta_cur " << theta1_cur << "      " << theta2_cur << std::endl;
//         if (result_found && (left_arm_joint_1->setTargetPosition(target_theta[0], 1)) && (left_arm_joint_2->setTargetPosition(target_theta[1], 1)))
//         {
//             return true;
//         }
//     }
//     else if (name == "right_arm")
//     {
//         static float min_sum_theta = PI * 2;
//         float min_theta[2] = {0, 0};
//         std::vector<std::vector<float>> theta = this->inverseKinematics(target);
//         simSetObjectInt32Parameter(handles.right_arm_joint_1, sim_jointintparam_ctrl_enabled, 1); // enable postion control
//         simSetObjectInt32Parameter(handles.right_arm_joint_2, sim_jointintparam_ctrl_enabled, 1);
//         for (int i = 0; i < theta.size(); i++)
//         {
//             float sum_theta = 0;
//             if (theta[i][1] > 0)
//             {
//                 sum_theta += abs(theta[i][0]) + abs(-theta[i][1]);
//                 if (sum_theta < min_sum_theta)
//                 {
//                     min_sum_theta = sum_theta;
//                     min_theta[0] = theta[i][0];
//                     min_theta[1] = theta[i][1];
//                 }
//             }
//             Eigen::Vector3f result = this->forwardKinematics(theta[i]);
//         }
//         if ((right_arm_joint_1->setTargetPosition(min_theta[0], 1)) && (right_arm_joint_2->setTargetPosition(min_theta[1], 1)))
//         {
//             return true;
//         }
//     }
//     return false;
// }

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