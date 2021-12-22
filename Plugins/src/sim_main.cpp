#include "sim_data.h"
#include "sim_main.h"
#include <iostream>
#include <math.h>
#include "/home/hao/CoppeliaSim_Edu_V4_2_0_Ubuntu18_04/programming/include/simLib.h"

void mainSimulation()
{
    getRobotInfo();
    getCustomerInfo();

    faceTracking();
    moveToCustomer();
}

void moveToCustomer()
{
    const float vel_set = 3;
    const float d = 0.27;
    const float bias = 3;

    Point p1(robot.x, robot.y);
    Point p2(robot.x + bias * cos(robot.yaw), robot.y + bias * sin(robot.yaw));
    Point p3(customer.x + bias * cos(customer.yaw), customer.y + bias * sin(customer.yaw));
    Point p4(customer.x, customer.y);

    float dist = sqrt(pow(customer.x - robot.x, 2) + pow(customer.y - robot.y, 2));
    float t = 0;
    std::cout << customer.yaw << std::endl;
    Point target;
    for (int i = 0; i < 100; i++)
    {
        target = p4 * (1 - t) * (1 - t) * (1 - t) + p3 * 3 * (1 - t) * (1 - t) * t + p2 * 3 * (1 - t) * t * t + p1 * t * t * t;
        t += 0.01;
        float pointToDraw[] = {target.x, target.y, 0};
        simAddDrawingObjectItem(handles.drawer, pointToDraw);
    }

    target = p4;
    float m = target.x - robot.x;
    float n = target.y - robot.y;
    float r = (m * m + n * n) / (2 * m * sin(robot.yaw) - 2 * n * cos(robot.yaw));
    float v1 = (r + d) / r * vel_set;
    float v2 = (r - d) / r * vel_set;

    simSetJointTargetVelocity(handles.left_wheel, v1);
    simSetJointTargetVelocity(handles.right_wheel, v2);
}

void faceTracking()
{
    const float kp = 5;
    const float ki = 0.01;
    const float kd = 0;
    const float tolerance = 0.02;
    const float vel_max = 1;

    static float integral_x = 0;
    static float integral_y = 0;
    static float error_x_last = 0;
    static float error_y_last = 0;

    float error_x = head.x - 0.5;
    float error_y = head.y - 0.5;

    float v1 = 0, v2 = 0;

    if (head.x >= 0 && head.y >= 0)
    {
        if (std::abs(error_x) > tolerance)
            v1 = kp * error_x + ki * integral_x + kd * (error_x - error_x_last);
        else
            integral_x = 0;
        if (std::abs(error_y) > tolerance)
            v2 = kp * error_y + ki * integral_y + kd * (error_y - error_y_last);
        else
            integral_y = 0;
    }

    integral_x += error_x;
    integral_y += error_y;
    error_x_last = error_x;
    error_y_last = error_y;

    v1 = std::min(std::max(v1, -vel_max), vel_max);
    v2 = std::min(std::max(v2, -vel_max), vel_max);

    simSetJointTargetVelocity(handles.waist_joint, -v1);
    simSetJointTargetVelocity(handles.head_joint_2, -v2);
}