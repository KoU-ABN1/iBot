#pragma once

#include <iostream>
#include <math.h>
#include <vector>
#include <Eigen/Dense>
#include <memory>

#include "/home/hao/CoppeliaSim_Edu_V4_2_0_Ubuntu18_04/programming/include/simLib.h"

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
    float robot_x;
    float robot_y;
    float robot_yaw;

    float customer_x;
    float customer_y;
    float customer_yaw;

    float head_x;
    float head_y;

    float time_cur;

    int table_number;
};

extern SimHandles handles;
extern SimInfo data;

void updateAllInfo();

void getObjectHandles(std::vector<int>);