#include "arm.h"

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
    float n = sin(theta[2]) * ARM_LENGTH;
    float p = -cos(theta[0]) * cos(theta[1]) * ARM_LENGTH;

    return Eigen::Vector3f(m, n, p);
}