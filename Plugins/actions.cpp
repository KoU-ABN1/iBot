#include "actions.h"

int Robot::waitAtDoor()
{
    const float WELCOME_DIS = 5;
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
    float pos[3]; // left_arm_base position
    simGetObjectPosition(handles.right_arm_joint_1, -1, pos);
    Eigen::Vector3f left_arm_base(pos[0], pos[1], pos[2]);
    Eigen::Vector3f target_abs(5, 0, 0); // target position in the world coordinate system

    Eigen::Vector3f target = target_abs - left_arm_base;

    std::cout << target[0] << "   " << target[1] << "   " << target[2] << std::endl;

    std::vector<std::vector<float>> theta = left_arm->inverseKinematics(target);

    simSetObjectInt32Parameter(handles.left_arm_joint_1, sim_jointintparam_ctrl_enabled, 1); // enable postion control
    simSetObjectInt32Parameter(handles.left_arm_joint_2, sim_jointintparam_ctrl_enabled, 1);

    for (int i = 0; i < theta.size(); i++)
    {
        if (theta[i][1] < 0)
        {
            simSetJointTargetPosition(handles.left_arm_joint_1, theta[i][0]);
            simSetJointTargetPosition(handles.left_arm_joint_2, -theta[i][1]);
        }
        Eigen::Vector3f result = left_arm->forwardKinematics(theta[i]);
        std::cout << theta[i][0] << "       " << theta[i][1] << std::endl;
        std::cout << result[0] << "      " << result[1] << "      " << result[2] << "      " << std::endl;
    }

    std::cout << std::endl;

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
            return INTERACT_WITH_CUSTOMER_AT_DOOR;
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