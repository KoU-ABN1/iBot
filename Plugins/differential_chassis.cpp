#include "differential_chassis.h"

Eigen::Vector2f DifferentialChassis::BezierPlanner()
{
    const float bias = 0.5;

    Eigen::Vector2f p1(robot.x, robot.y);
    Eigen::Vector2f p2(robot.x + bias * cos(robot.yaw), robot.y + bias * sin(robot.yaw));
    Eigen::Vector2f p3(customer.x + bias * cos(customer.yaw), customer.y + bias * sin(customer.yaw));
    Eigen::Vector2f p4(customer.x, customer.y);

    float dist = sqrt(pow(customer.x - robot.x, 2) + pow(customer.y - robot.y, 2));
    float t = 0.1;
    Eigen::Vector2f target = p1 * (1 - t) * (1 - t) * (1 - t) + p2 * 3 * (1 - t) * (1 - t) * t + p3 * 3 * (1 - t) * t * t + p4 * t * t * t;

    static bool flag = true;
    if (flag)
    {
        float t = 0;
        for (int i = 0; i < 100; i++)
        {
            Eigen::Vector2f point = p1 * (1 - t) * (1 - t) * (1 - t) + p2 * 3 * (1 - t) * (1 - t) * t + p3 * 3 * (1 - t) * t * t + p4 * t * t * t;
            t += (float)1 / 100;
            float pointToDraw[] = {point[0], point[1], 0};
            simAddDrawingObjectItem(handles.drawer, pointToDraw);
        }
        flag = true;
    }

    // float pointToDraw[] = {target[0], target[1], 0};
    // simAddDrawingObjectItem(handles.drawer, pointToDraw);

    return target;
}

void DifferentialChassis::moveToPointWithArc(const Eigen::Vector2f &target)
{
    float m = target[0] - robot.x;
    float n = target[1] - robot.y;
    float r = (m * m + n * n) / (2 * m * sin(robot.yaw) - 2 * n * cos(robot.yaw));
    float v1 = (r + D) / r * VEL_SET;
    float v2 = (r - D) / r * VEL_SET;

    left_wheel->setVelocity(v1);
    right_wheel->setVelocity(v2);

    setWheelVelocity(v1, v2);
}

void DifferentialChassis::stop()
{
    setWheelVelocity(0, 0);
}

void DifferentialChassis::setWheelVelocity(const float v1, const float v2)
{
    left_wheel->setVelocity(v1);
    right_wheel->setVelocity(v2);
}