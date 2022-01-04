#pragma once

#include "common.h"
#include "motor.h"

class Chassis
{
public:
    /**
     * @brief Control the chassis to move to the customer
     * 
     * @param vel Target velocity
     * @param acc Maximum acceleration (set to negative to disable it)
     */
    void moveToCustomer(const float vel, const float acc = ACC_DEFAULT);

    /**
     * @brief Control the chassis to move to the table
     * 
     * @param vel Target velocity
     * @param acc Maximum acceleration (set to negative to disable it)
     */
    void moveToTable(const std::vector<Eigen::Vector2f> &nodes, const float vel, const float acc = ACC_DEFAULT);

    void moveToDoor(const std::vector<Eigen::Vector2f> &nodes, const float vel, const float acc = ACC_DEFAULT);

    /**
     * @brief Control the chassis to move to a point in x-y plane with a circle path
     * 
     * @param target Target point in x-y plane
     * @param vel Taret velocity
     * @param acc Maximum acceleration (set to negative to disable it)
     */
    void moveToPointWithArc(const Eigen::Vector2f &target, const float vel, const float acc = ACC_DEFAULT);

    /**
     * @brief Control the chssis to stop
     * 
     * @param acc Maximum acceleration (set to negative to disable it)
     */
    void stop(const float acc = ACC_DEFAULT);

    /**
     * @brief Control the chassis to rotate in place at a certain speed
     * 
     * @param vel Target velocity
     * @param acc Target acceleration (set to negative to disable it)
     */
    void rotateInPlaceVelocity(const float vel, const float acc = ACC_DEFAULT);

    /**
     * @brief Rotate chassis to the target position (yaw)
     * 
     * @param target_yaw Target position (yaw)
     * @param vel Target velocity
     * @param acc Maximum accelerarion
     * @return True if chassis has rotated to the target position (yaw)
     * @return False if chassis has not rotated to the target position (yaw)
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
