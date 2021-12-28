#include "actions.h"

int Robot::waitAtDoor()
{
    const float WELCOME_DIS = 5;
    float dist = sqrt(pow(data.robot_x - data.customer_x, 2) + pow(data.robot_y - data.customer_y, 2));

    if (dist < WELCOME_DIS)
        return MOVE_TO_CUSTOMER;
    else
        return WAIT_AT_DOOR;
}

int Robot::moveToCustomer()
{
    const float STOP_DIS = 1.2;
    float dist = sqrt(pow(data.robot_x - data.customer_x, 2) + pow(data.robot_y - data.customer_y, 2));

    if (dist > STOP_DIS)
    {
        chassis->moveToCustomer(3, 1);
        body->trackCustomerFace();

        return MOVE_TO_CUSTOMER;
    }
    else
    {
        body->trackCustomerFace();

        float vel_set = 0.5;

        const float D1 = 300.0;
        const float D2 = 540.0;

        float v1 = vel_set * D2 / D1;
        float v2 = vel_set;

        if (data.waist_joint_position > 5.0 / 180 * 3.14)
        {
            simSetJointTargetVelocity(handles.left_wheel, -v1);
            simSetJointTargetVelocity(handles.right_wheel, v1);
            simSetJointTargetVelocity(handles.waist_joint, -v2);
        }
        else
        {
            simSetJointTargetVelocity(handles.left_wheel, 0);
            simSetJointTargetVelocity(handles.right_wheel, 0);
            simSetJointTargetVelocity(handles.waist_joint, 0);

            WAIT(1, INTERACT_WITH_CUSTOMER_AT_DOOR);
        }

        return MOVE_TO_CUSTOMER;
    }
}

int Robot::interactWithCustomerAtDoor()
{
    // Eigen::Vector3f target;   // target position
    // Eigen::Vector3f arm_base; // arm_base position

    // float m = target[0] - arm_base[0];
    // float n = target[1] - arm_base[1];
    // float p = target[2] - arm_base[2];

    // float theta2 = atan2(n, p);
    // float r = m * sin(theta2);
    // float theta1 = atan2(r, n);

    // simSetObjectInt32Parameter(handles.left_arm_joint_1, sim_jointintparam_ctrl_enabled, 1); // enable postion control
    // simSetObjectInt32Parameter(handles.left_arm_joint_2, sim_jointintparam_ctrl_enabled, 1);

    // simSetJointTargetPosition(handles.left_arm_joint_1, theta1);
    // simSetJointTargetPosition(handles.left_arm_joint_2, theta2);

    WAIT(1, GET_TABLE_NUMBER);

    return INTERACT_WITH_CUSTOMER_AT_DOOR;
}

int Robot::getTableNumber()
{
    simSetObjectInt32Parameter(handles.head_joint_2, sim_jointintparam_ctrl_enabled, 1); // enable postion control
    simSetObjectFloatParameter(handles.head_joint_2, sim_jointfloatparam_upper_limit, 1);
    simSetJointTargetPosition(handles.head_joint_2, 0);

    data.table_number = 7;

    return TAKE_CUSTOMER_TO_TABLE;
}

int Robot::takeCustomerToTable()
{
    static int substate = 0;

    switch (substate)
    {
    case 0:
    {
        float tolerance = 0.1;
        float target_yaw = atan2(data.door_y - data.robot_y, data.door_x - data.robot_x);
        float yaw_diff = rectifyAngle(target_yaw - data.robot_yaw);
        std::cout << target_yaw << std::endl;
        if (yaw_diff > tolerance)
            chassis->rotateInPlace(3);
        else if (yaw_diff < -tolerance)
            chassis->rotateInPlace(-3);
        else
        {
            chassis->stop(1);
            substate = 1;
        }

        break;
    }
    case 1:
    {
        chassis->moveToTable(3);
        Eigen::Vector2f target(1.5, -6);
        float dist = sqrt(pow(target[0] - data.robot_x, 2) + pow(target[1] - data.robot_y, 2));
        if (dist < 0.2)
            substate = 1;
        break;
    }

    case 2:
    {
        chassis->stop(1);
        break;
    }
    }

    return TAKE_CUSTOMER_TO_TABLE;
}

int Robot::interactWithCustomerAtTable()
{

    return BACK_TO_DOOR;
}

int Robot::backToDoor()
{

    return WAIT_AT_DOOR;
}