#pragma once

#include "common.h"
#include "motor.h"

class DifferentialChassis
{
public:
    Point BezierPlanner();
    void moveToPoint(const Point &target);
    void setWheelVelocity(const float v1, const float v2);

private:
    const float D = 0.27;
    const float VEL_SET = 3;

    Motor *left_wheel = new Motor(handles.left_wheel);

    //std::unique_ptr<Motor> left_wheel = std::make_unique<Motor>(Motor(handles.left_wheel));
    std::unique_ptr<Motor> right_wheel = std::make_unique<Motor>(Motor(handles.right_wheel));
};
