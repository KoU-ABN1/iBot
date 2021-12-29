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
    head_joint_2->setTargetPosition(0, 0.5);

    data.table_number = 7;

    return TAKE_CUSTOMER_TO_TABLE;
}

int Robot::takeCustomerToTable()
{
    static int substate = FACE_TO_DOOR;
    float vel_set = 3;

    switch (substate)
    {
    case FACE_TO_DOOR:
    {
        float tolerance = 0.1;
        float target_yaw = atan2(data.door_y - data.robot_y, data.door_x - data.robot_x);
        float yaw_diff = rectifyAngle(target_yaw - data.robot_yaw);
        if (yaw_diff > tolerance)
            chassis->rotateInPlace(vel_set);
        else if (yaw_diff < -tolerance)
            chassis->rotateInPlace(-vel_set);
        else
        {
            chassis->stop(1);
            substate = MOVE_TO_TABLE;
        }

        break;
    }
    case MOVE_TO_TABLE:
    {
        chassis->moveToTable(vel_set);
        float dist = sqrt(pow(data.table_x - data.robot_x, 2) + pow(data.table_y - data.robot_y, 2));
        if (dist < 0.2)
            substate = STOP_AT_TABLE;
        break;
    }

    case STOP_AT_TABLE:
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