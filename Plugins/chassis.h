#pragma once

#include "common.h"
#include "motor.h"

class Chassis
{
public:
    /**
     * @brief Control the chassis to move to the customer
     * 
     * @param vel target velocity
     * @param acc maximum acceleration (set to negative to disable it)
     */
    void moveToCustomer(const float vel, const float acc = ACC_DEFAULT);

    /**
     * @brief Control the chassis to move to the table
     * 
     * @param vel target velocity
     * @param acc maximum acceleration (set to negative to disable it)
     */
    void moveToTable(const float vel, const float acc = ACC_DEFAULT);

    /**
     * @brief Control the chassis to move to a point in x-y plane with a circle path
     * 
     * @param target target point in x-y plane
     * @param vel taret velocity
     * @param acc maximum acceleration (set to negative to disable it)
     */
    void moveToPointWithArc(const Eigen::Vector2f &target, const float vel, const float acc = ACC_DEFAULT);

    /**
     * @brief Control the chssis to stop
     * 
     * @param acc maximum acceleration (set to negative to disable it)
     */
    void stop(const float acc = ACC_DEFAULT);

    /**
     * @brief Control the chassis to rotate in place at a certain speed
     * 
     * @param vel target velocity
     * @param acc target acceleration (set to negative to disable it)
     */
    void rotateInPlaceVelocity(const float vel, const float acc = ACC_DEFAULT);

    /**
     * @brief Rotate chassis to the target position (yaw)
     * 
     * @param target_yaw target position (yaw)
     * @param vel target velocity
     * @param acc maximum accelerarion
     * @return true if chassis has rotated to the target position
     * @return false if chassis has not rotated to the target position
     */
    bool rotateInPlacePosition(const float target_yaw, const float vel, const float acc = ACC_DEFAULT);

private:
    constexpr static float ACC_DEFAULT = -1;
    const float TRACK_WIDTH = 0.27;

    std::unique_ptr<Motor> left_wheel = std::make_unique<Motor>(Motor(handles.left_wheel));
    std::unique_ptr<Motor> right_wheel = std::make_unique<Motor>(Motor(handles.right_wheel));

    std::vector<Eigen::Vector2f> generateWaypoints(const std::vector<Eigen::Vector2f> &nodes);

    void drawWaypoints(const std::vector<Eigen::Vector2f> &waypoints);
};
