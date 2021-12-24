#include "differential_chassis.h"

Point DifferentialChassis::BezierPlanner()
{
    const float bias = 0.5;

    Point p1(robot.x, robot.y);
    Point p2(robot.x + bias * cos(robot.yaw), robot.y + bias * sin(robot.yaw));
    Point p3(customer.x + bias * cos(customer.yaw), customer.y + bias * sin(customer.yaw));
    Point p4(customer.x, customer.y);

    float dist = sqrt(pow(customer.x - robot.x, 2) + pow(customer.y - robot.y, 2));
    float t = 1;
    Point target = p1 * (1 - t) * (1 - t) * (1 - t) + p2 * 3 * (1 - t) * (1 - t) * t + p3 * 3 * (1 - t) * t * t + p4 * t * t * t;

    float pointToDraw[] = {target.x, target.y, 0};
    simAddDrawingObjectItem(handles.drawer, pointToDraw);

    return target;
}

void DifferentialChassis::moveToPoint(const Point &target)
{
    float m = target.x - robot.x;
    float n = target.y - robot.y;
    float r = (m * m + n * n) / (2 * m * sin(robot.yaw) - 2 * n * cos(robot.yaw));
    float v1 = (r + D) / r * VEL_SET;
    float v2 = (r - D) / r * VEL_SET;

    left_wheel->setVelocity(v1);
    right_wheel->setVelocity(v2);

    setWheelVelocity(v1, v2);
}

void DifferentialChassis::setWheelVelocity(const float v1, const float v2)
{
    std::cout << "chassis        " << v1 << "    " << v2 << std::endl;
    left_wheel->setVelocity(v1);
    right_wheel->setVelocity(v2);
}