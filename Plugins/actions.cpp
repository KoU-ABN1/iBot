#include "actions.h"

int Robot::waitAtDoor()
{
    const float WELCOME_DIS = 10;
    float dist = sqrt(pow(data.robot_x - data.customer_x, 2) + pow(data.robot_y - data.customer_y, 2));

    if (dist < WELCOME_DIS)
        return MOVE_TO_CUSTOMER;

    return WAIT_AT_DOOR;
}

int Robot::moveToCustomer()
{
    static int substate = GET_CLOSE_TO_CUSTOMER;

    switch (substate)
    {
    case GET_CLOSE_TO_CUSTOMER:
    {
        const float STOP_DIS = 1.2;
        float dist = sqrt(pow(data.robot_x - data.customer_x, 2) + pow(data.robot_y - data.customer_y, 2));

        if (dist > STOP_DIS)
        {
            chassis->moveToCustomer(3, 1);
            body->trackCustomerFace();
        }
        else
            substate = ADJUST_POSTURE;
        break;
    }

    case ADJUST_POSTURE:
    {
        float vel_set = 0.6;
        float tolerance = 2.0 / 180.0 * PI;
        float vel_wheel = vel_set * TRACK_WIDTH / WHEEL_DIAMETER;

        if (data.waist_joint_position > tolerance)
        {
            chassis->rotateInPlaceVelocity(vel_wheel);
            body->trackCustomerFace(vel_set, 2 * vel_set, 0.1, 0.01, 0);
        }
        else
        {
            chassis->stop();
            body->stop();
            return INTERACT_WITH_CUSTOMER_AT_DOOR;
        }
        break;
    }
    }

    return MOVE_TO_CUSTOMER;
}

int Robot::interactWithCustomerAtDoor()
{

    Eigen::Vector3f target_abs(0, 0, 0);
    left_arm->pointToTargetPosition(target_abs);

    WAIT(5, GET_TABLE_NUMBER);

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

    switch (substate)
    {
    case FACE_TO_DOOR:
    {
        float vel_set = 3;
        float target_yaw = atan2(data.door_y - data.robot_y, data.door_x - data.robot_x);

        if (chassis->rotateInPlacePosition(target_yaw, vel_set))
            substate = MOVE_TO_TABLE;
        break;
    }

    case MOVE_TO_TABLE:
    {
        float vel_set = 3;
        const float STOP_DIS = 0.2;

        chassis->moveToTable(vel_set);

        float dist = sqrt(pow(data.table_x - data.robot_x, 2) + pow(data.table_y - data.robot_y, 2));
        if (dist < STOP_DIS)
            substate = FACE_TO_CUSTOMER;
        break;
    }

    case FACE_TO_CUSTOMER:
    {
        float vel_set = 2;

        float target_yaw = atan2(data.customer_y - data.robot_y, data.customer_x - data.robot_x);
        if (chassis->rotateInPlacePosition(target_yaw, vel_set))
            return INTERACT_WITH_CUSTOMER_AT_TABLE;
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