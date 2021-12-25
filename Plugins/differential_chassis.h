#pragma once

#include "common.h"
#include "motor.h"

class DifferentialChassis
{
public:
    Eigen::Vector2f BezierPlanner();

    void moveToPointWithArc(const Eigen::Vector2f &target);
    void moveToPointWithLine(const Eigen::Vector2f &target);
    void stop();

    void rotateCounterclockwise();
    void rotateClockwise();

private:
    const float D = 0.27;
    const float VEL_SET = 3;

    std::unique_ptr<Motor> left_wheel = std::make_unique<Motor>(Motor(handles.left_wheel));
    std::unique_ptr<Motor> right_wheel = std::make_unique<Motor>(Motor(handles.right_wheel));
};
