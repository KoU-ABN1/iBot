#include "differential_chassis.h"

Eigen::Vector2f DifferentialChassis::BezierPlanner()
{
    const float bias = 1;

    Eigen::Vector2f p1(data.robot_x, data.robot_y);
    Eigen::Vector2f p2(data.robot_x + bias * cos(data.robot_yaw), data.robot_y + bias * sin(data.robot_yaw));
    Eigen::Vector2f p3(data.customer_x + bias * cos(data.customer_yaw), data.customer_y + bias * sin(data.customer_yaw));
    Eigen::Vector2f p4(data.customer_x, data.customer_y);

    float dist = sqrt(pow(data.robot_x - data.customer_x, 2) + pow(data.robot_y - data.customer_y, 2));
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
        flag = false;
    }

    // float pointToDraw[] = {target[0], target[1], 0};
    // simAddDrawingObjectItem(handles.drawer, pointToDraw);

    return target;
}

void DifferentialChassis::moveToPointWithArc(const Eigen::Vector2f &target)
{
    float m = target[0] - data.robot_x;
    float n = target[1] - data.robot_y;
    float r = (m * m + n * n) / (2 * m * sin(data.robot_yaw) - 2 * n * cos(data.robot_yaw));
    float v1 = (r + D) / r * VEL_SET;
    float v2 = (r - D) / r * VEL_SET;

    float move_acc = 0.5;
    left_wheel->setAccMax(move_acc);
    right_wheel->setAccMax(move_acc);

    left_wheel->setVelocity(v1);
    right_wheel->setVelocity(v2);
}

void DifferentialChassis::stop()
{
    float brake_acc = 3;
    left_wheel->setAccMax(brake_acc);
    right_wheel->setAccMax(brake_acc);

    left_wheel->setVelocity(0);
    right_wheel->setVelocity(0);
}

void DifferentialChassis::rotateCounterclockwise()
{
    float move_acc = 10;
    float vel_set = 0.5;

    left_wheel->setAccMax(move_acc);
    right_wheel->setAccMax(move_acc);

    left_wheel->setVelocity(-vel_set);
    right_wheel->setVelocity(vel_set);
}

void DifferentialChassis::rotateClockwise()
{
    float move_acc = 10;
    float vel_set = 0.5;

    left_wheel->setAccMax(move_acc);
    right_wheel->setAccMax(move_acc);

    left_wheel->setVelocity(vel_set);
    right_wheel->setVelocity(-vel_set);
}