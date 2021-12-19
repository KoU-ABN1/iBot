#include "sim_data.h"
#include "/home/hao/CoppeliaSim_Edu_V4_2_0_Ubuntu18_04/programming/include/simLib.h"

void mainSimulation() {
    getRobotInfo();
    getCustomerInfo();


    simSetJointTargetVelocity(handles.left_wheel, 10);
}