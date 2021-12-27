#include "actions.h"

int Robot::waitAtDoor()
{
    const float welcome_dist = 5;

    float dist = sqrt(pow(data.robot_x - data.customer_x, 2) + pow(data.robot_y - data.customer_y, 2));

    if (dist < welcome_dist && data.head_x >= 0 && data.head_y >= 0)
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
        Eigen::Vector2f target = chassis->BezierPlanner();
        chassis->moveToPointWithArc(target);

        body->trackCustomerFace();

        return MOVE_TO_CUSTOMER;
    }
    else
    {
        body->trackCustomerFace();

        Motor left_wheel(handles.left_wheel);
        Motor right_wheel(handles.right_wheel);
        Motor waist_joint(handles.waist_joint);

        float vel_set = 0.5;

        float d1 = 300.0;
        float d2 = 540.0;

        float v1 = vel_set * d2 / d1;
        float v2 = vel_set;

        std::cout << data.waist_joint_position << std::endl;

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
    WAIT(3, GET_TABLE_NUMBER);

    return INTERACT_WITH_CUSTOMER_AT_DOOR;
}

int Robot::getTableNumber()
{
    data.table_number = 8;

    return TAKE_CUSTOMER_TO_TABLE;
}

int Robot::takeCustomerToTable()
{

    return INTERACT_WITH_CUSTOMER_AT_TABLE;
}

int Robot::interactWithCustomerAtTable()
{

    return BACK_TO_DOOR;
}

int Robot::backToDoor()
{

    return WAIT_AT_DOOR;
}