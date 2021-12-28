#pragma once

#include "common.h"
#include "motor.h"

class DifferentialChassis
{
public:
    void moveToCustomer(const float vel_set);

    void moveToTable(const float vel_set);

    Eigen::Vector2f BezierPlanner();

    void moveToPointWithArc(const Eigen::Vector2f &target, const float vel_set, const float acc_max);

    void stop(const float acc_max);

    void rotateCounterclockwise();

    void rotateClockwise();

private:
    const float D = 0.27;
    const float VEL_SET = 3;

    std::unique_ptr<Motor> left_wheel = std::make_unique<Motor>(Motor(handles.left_wheel));
    std::unique_ptr<Motor> right_wheel = std::make_unique<Motor>(Motor(handles.right_wheel));

    std::vector<Eigen::Vector2f> generateWaypoints(const std::vector<Eigen::Vector2f> &nodes);
    void drawWaypoints(const std::vector<Eigen::Vector2f> &waypoints);
};
