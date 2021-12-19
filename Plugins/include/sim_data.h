#pragma once

#include <vector>
#include "/home/hao/CoppeliaSim_Edu_V4_2_0_Ubuntu18_04/programming/include/simLib.h"

struct SimHandles {
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
};

struct RobotInfo {
    float x;
    float y;
    float yaw;
};

struct CustomerInfo {
    float x;
    float y;
    float yaw;
    float head_x;
    float head_y;
};

extern SimHandles handles;
extern RobotInfo robot;
extern CustomerInfo customer;

void getObjectHandles(std::vector<int>);
void getRobotInfo();
void getCustomerInfo();