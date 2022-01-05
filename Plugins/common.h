#pragma once

#include <iostream>
#include <math.h>
#include <vector>
#include </usr/include/eigen3/Eigen/Dense>
#include <memory>

#include "/home/xiayu/CoppeliaSim_Edu_V4_2_0_Ubuntu18_04/programming/include/simLib.h"

#define NOTICE(text)                        \
    {                                       \
        static bool flag = true;            \
        if (flag)                           \
        {                                   \
            std::cout << text << std::endl; \
            flag = false;                   \
        }                                   \
    }

#define WAIT(t, s)                               \
    {                                            \
        static float time_start = data.time_cur; \
        if (data.time_cur - time_start > t)      \
            return s;                            \
    }

#define PI 3.14159265

struct SimHandles
{
    int left_wheel;
    int right_wheel;
    int waist_joint;
    int head_joint_1;
    int head_joint_2;
    int left_arm_joint_1;
    int left_arm_joint_2;
    int right_arm_joint_1;
    int right_arm_joint_2;

    int robot;
    int customer;

    int drawer;
};

struct SimInfo
{
    float robot_x; // robot coordinates
    float robot_y;
    float robot_yaw;

    float customer_x; // customer coordinates
    float customer_y;
    float customer_yaw;

    float head_x; // position of the customer's head in the camera
    float head_y;

    float waist_joint_position; // joint positions
    float head_joint_1_position;
    float head_joint_2_position;
    float left_arm_joint_1_position;
    float left_arm_joint_2_position;
    float right_arm_joint_1_position;
    float right_arm_joint_2_position;

    float time_cur; // current time

    int table_number; // target table number

    float door_x = 4;
    float door_y = 0;

    float table_x = 1.5;
    float table_y = -6;
};

extern SimHandles handles;
extern SimInfo data;

void updateAllInfo();
void getObjectHandles(std::vector<int>);

inline float rectifyAngle(float angle)
{
    if (angle > PI)
        angle -= 2 * PI;
    else if (angle <= -PI)
        angle += 2 * PI;
    return angle;
}