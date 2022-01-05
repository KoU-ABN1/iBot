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
    //门的位置
    Eigen::Vector3f target(4, 0, 1);

    //机器人平面内转向门店所需的角度
    float target_yaw = atan2(target[1] - data.robot_y, target[0] - data.robot_x);
    float scale = 0.5; //转向所需角度的1/2
    float waist_yaw = data.robot_yaw + data.waist_joint_position;
    static float yaw_diff = rectifyAngle(target_yaw - waist_yaw) * scale;

    static int substate = TURN_WAIST_TO_DOOR;
    static int times = 0;
    static int hands_time = 0;

    float face_target_yaw = atan2(data.customer_y - data.robot_y, data.customer_x - data.robot_x);
    head_joint_1->setTargetPosition(data.waist_joint_position, 2);

    head_joint_2->setTargetPosition(-30 / 180.0 * PI, 1);

    switch (substate)
    {
    case TURN_WAIST_TO_DOOR:
    {
        std::cout << "ttt" << std::endl;

        if (waist_joint->setTargetPosition(yaw_diff, 1))
        {
            substate = HANDS_UP;
            break;
        }
        break;
    }
    case HANDS_UP:
    {
        static bool flag = true;

        if ((rectifyAngle(target_yaw) - rectifyAngle(waist_yaw)) > 0)
        {
            hands_time++;
            if (flag)
            {
                left_arm->pointToTargetPosition(target, 3);
                times++;
                flag = false;
            }
        }
        if (hands_time == 300)
        {
            substate = HANDES_DOWN;
            flag = true;
        }

        break;
    }
    case HANDES_DOWN:
    {
        if (times == 3)
        {
            return GET_TABLE_NUMBER;
        }

        hands_time--;
        if (hands_time == 0)
        {
            substate = HANDS_UP;
        }

        left_arm_joint_1->setTargetPosition(0, 3);
        left_arm_joint_2->setTargetPosition(0, 3);

        //substate = HANDS_UP;

        break;
    }

    default:
        break;
    }

    return INTERACT_WITH_CUSTOMER_AT_DOOR;
}

int Robot::getTableNumber()
{
    static float getTableNumberTime = 0;
    Eigen::Vector3f arm_reset(0, 0, -1);

    waist_joint->setTargetPosition(0, 1);
    head_joint_2->setTargetPosition(0, 1);
    head_joint_1->setTargetPosition(0, 1);

    left_arm_joint_1->setTargetPosition(0, 1);
    left_arm_joint_2->setTargetPosition(0, 1);

    data.table_number = 7;
    getTableNumberTime++;
    if (getTableNumberTime = 1000)
    {
        return TAKE_CUSTOMER_TO_TABLE;
    }
    return GET_TABLE_NUMBER;
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

        std::vector<Eigen::Vector2f> nodes;
        nodes.push_back(Eigen::Vector2f(data.robot_x, data.robot_y));
        nodes.push_back(Eigen::Vector2f(4, 0));
        nodes.push_back(Eigen::Vector2f(4, -1));
        nodes.push_back(Eigen::Vector2f(1.5, -1));
        nodes.push_back(Eigen::Vector2f(1.5, -6));

        chassis->moveToTable(nodes, vel_set);

        float dist = sqrt(pow(data.table_x - data.robot_x, 2) + pow(data.table_y - data.robot_y, 2));
        if (dist < STOP_DIS)
            substate = FACE_TO_CUSTOMER;
        break;
    }

    case FACE_TO_CUSTOMER:
    {
        float vel_set = 2;

        if (chassis->rotateInPlacePosition(3.14 / 2, vel_set))
            return INTERACT_WITH_CUSTOMER_AT_TABLE;

        break;
    }
    }

    return TAKE_CUSTOMER_TO_TABLE;
}

int Robot::interactWithCustomerAtTable()
{
    static float table_time = 0;
    static bool table_flag = true;
    Eigen::Vector3f table_target(-1.5, -6, 0.5);
    //Eigen::Vector3f table_target(0, 0, 0);
    if (table_flag)
    {
        left_arm->pointToTargetPosition(table_target, 2);
        table_flag = false;
    }
    table_time++;
    if (table_time == 300)
    {
        left_arm_joint_1->setTargetPosition(0, 2);
        left_arm_joint_2->setTargetPosition(0, 2);
        return BACK_TO_DOOR;
    }
    return INTERACT_WITH_CUSTOMER_AT_TABLE;
}

int Robot::backToDoor()
{

    std::vector<Eigen::Vector2f> nodes;
    nodes.push_back(Eigen::Vector2f(data.robot_x, data.robot_y));
    nodes.push_back(Eigen::Vector2f(1.5, -1));
    nodes.push_back(Eigen::Vector2f(4, -1));
    nodes.push_back(Eigen::Vector2f(4, 0));
    nodes.push_back(Eigen::Vector2f(3, 1));

    chassis->moveToDoor(nodes, 3);

    const float STOP_DIS = 0.2;
    float dist = sqrt(pow(3 - data.robot_x, 2) + pow(1 - data.robot_y, 2));
    if (dist < STOP_DIS)
    {
        chassis->stop();
        return WAIT_AT_DOOR;
    }

    return BACK_TO_DOOR;
}