#pragma once

#include "common.h"
#include "chassis.h"
#include "body.h"
#include "arm.h"

enum Actions
{
    WAIT_AT_DOOR,
    MOVE_TO_CUSTOMER,
    INTERACT_WITH_CUSTOMER_AT_DOOR,
    GET_TABLE_NUMBER,
    TAKE_CUSTOMER_TO_TABLE,
    INTERACT_WITH_CUSTOMER_AT_TABLE,
    BACK_TO_DOOR,
};

enum Substate_MoveToCustomer
{
    GET_CLOSE_TO_CUSTOMER,
    ADJUST_POSTURE,
};

enum Substate_TakeCustomerToTable
{
    FACE_TO_DOOR,
    MOVE_TO_TABLE,
    FACE_TO_CUSTOMER,
};

class Robot
{
public:
    int waitAtDoor();
    int moveToCustomer();
    int interactWithCustomerAtDoor();
    int getTableNumber();
    int takeCustomerToTable();
    int interactWithCustomerAtTable();
    int backToDoor();

private:
    const float TRACK_WIDTH = 0.54;
    const float WHEEL_DIAMETER = 0.3;

    std::unique_ptr<Chassis> chassis = std::make_unique<Chassis>(Chassis());
    std::unique_ptr<Body> body = std::make_unique<Body>(Body());
    std::unique_ptr<Arm> left_arm = std::make_unique<Arm>(Arm());
    std::unique_ptr<Arm> right_arm = std::make_unique<Arm>(Arm());

    std::unique_ptr<Motor> left_wheel = std::make_unique<Motor>(Motor(handles.left_wheel));
    std::unique_ptr<Motor> right_wheel = std::make_unique<Motor>(Motor(handles.right_wheel));
    std::unique_ptr<Motor> waist_joint = std::make_unique<Motor>(Motor(handles.waist_joint));
    std::unique_ptr<Motor> head_joint_1 = std::make_unique<Motor>(Motor(handles.head_joint_1));
    std::unique_ptr<Motor> head_joint_2 = std::make_unique<Motor>(Motor(handles.head_joint_2));
};
