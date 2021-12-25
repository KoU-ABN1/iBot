#pragma once

#include "common.h"
#include "differential_chassis.h"
#include "three_part_body.h"
#include "two_dof_arm.h"

enum Actions
{
    WAIT_AT_DOOR,
    MOVE_TO_CUSTOMER,
    INTERACT_WITH_CUSTOMER_AT_DOOR,
    GET_TABLE_NUMBER,
    TAKE_CUSTOMER_TO_TABLE,
    INTERACT_WITH_CUSTOMER_AT_TABLE,
    BACK_TO_DOOR
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
    std::unique_ptr<DifferentialChassis> chassis = std::make_unique<DifferentialChassis>(DifferentialChassis());
    std::unique_ptr<ThreePartBody> body = std::make_unique<ThreePartBody>(ThreePartBody());
    std::unique_ptr<TwoDofArm> left_arm = std::make_unique<TwoDofArm>(TwoDofArm());
    std::unique_ptr<TwoDofArm> right_arm = std::make_unique<TwoDofArm>(TwoDofArm());
};
