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

typedef Eigen::Vector2f Point2f;

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

struct RobotInfo
{
    float x;
    float y;
    float yaw;
};

struct CustomerInfo
{
    float x;
    float y;
    float yaw;
};

struct VisionInfo
{
    float x;
    float y;
};

struct Point
{
    float x;
    float y;

    Point() {}
    Point(float t1, float t2) : x(t1), y(t2) {}

    Point operator+(Point p)
    {
        return Point(p.x + x, p.y + y);
    }
    Point operator*(float k)
    {
        return Point(k * x, k * y);
    }
};

extern SimHandles handles;
extern RobotInfo robot;
extern CustomerInfo customer;
extern VisionInfo head;
extern float time_cur;

void updateAllInfo();
void updateRobotInfo();
void updateCustomerInfo();
void updateTime();

void getObjectHandles(std::vector<int>);
