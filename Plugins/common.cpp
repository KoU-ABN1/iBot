#include "common.h"

SimHandles handles;
RobotInfo robot;
CustomerInfo customer;
VisionInfo head; // updated by sim.sendVisionInfo in sim_plugin.cpp
float time_cur;

void updateAllInfo()
{
    updateCustomerInfo();
    updateRobotInfo();
    updateTime();
}

void updateCustomerInfo()
{
    float position[3];
    float orientation[3];
    simGetObjectPosition(handles.customer, -1, position);
    simGetObjectOrientation(handles.customer, -1, orientation);
    customer.x = position[0];
    customer.y = position[1];
    customer.yaw = orientation[2];
}

void updateRobotInfo()
{
    float position[3];
    float orientation[3];
    simGetObjectPosition(handles.robot, -1, position);
    simGetObjectOrientation(handles.robot, -1, orientation);
    robot.x = position[0];
    robot.y = position[1];
    robot.yaw = orientation[2];
}

void updateTime()
{
    time_cur = simGetSimulationTime();
}

void getObjectHandles(std::vector<int> data)
{
    handles.left_wheel = data[0];
    handles.right_wheel = data[1];
    handles.waist_joint = data[2];
    handles.head_joint_1 = data[3];
    handles.head_joint_2 = data[4];
    handles.left_arm_joint_1 = data[5];
    handles.left_arm_joint_2 = data[6];
    handles.right_arm_joint_1 = data[7];
    handles.right_arm_joint_2 = data[8];
    handles.robot = data[9];
    handles.customer = data[10];
    handles.drawer = data[11];
}