#include "sim_data.h"
#include <iostream>
#include "/home/hao/CoppeliaSim_Edu_V4_2_0_Ubuntu18_04/programming/include/simLib.h"

void mainSimulation() {
    getRobotInfo();
    getCustomerInfo();

    std::cout << head.x << std::endl;

    simSetJointTargetVelocity(handles.left_wheel, 1);
}