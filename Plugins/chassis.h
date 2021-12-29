#pragma once

#include "common.h"
#include "motor.h"

class Chassis
{
public:
    void moveToCustomer(const float vel, const float acc = ACC_DEFAULT);

    void moveToTable(const float vel, const float acc = ACC_DEFAULT);

    void moveToPointWithArc(const Eigen::Vector2f &target, const float vel, const float acc = ACC_DEFAULT);

    void stop(const float acc = ACC_DEFAULT);

    void rotateInPlace(const float vel, const float acc = ACC_DEFAULT);

private:
    constexpr static float ACC_DEFAULT = -1;

    const float TRACK_WIDTH = 0.27;

    std::unique_ptr<Motor> left_wheel = std::make_unique<Motor>(Motor(handles.left_wheel));
    std::unique_ptr<Motor> right_wheel = std::make_unique<Motor>(Motor(handles.right_wheel));

    std::vector<Eigen::Vector2f> generateWaypoints(const std::vector<Eigen::Vector2f> &nodes);

    void drawWaypoints(const std::vector<Eigen::Vector2f> &waypoints);
};
