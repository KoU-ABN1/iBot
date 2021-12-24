#pragma once

#include "common.h"
#include "differential_chassis.h"
#include "three_part_body.h"
#include "two_dof_arm.h"

class Robot
{
public:
    void waitAtDoor();
    void moveToCustomer();
    void interactWithCustomerAtDoor();
    void getTableNumber();
    void takeCustomerToTable();
    void interactWithCustomerAtTable();
    void backToDoor();

private:
    std::unique_ptr<DifferentialChassis> chassis = std::make_unique<DifferentialChassis>(DifferentialChassis());
    std::unique_ptr<ThreePartBody> body = std::make_unique<ThreePartBody>(ThreePartBody());
    std::unique_ptr<TwoDofArm> left_arm = std::make_unique<TwoDofArm>(TwoDofArm());
    std::unique_ptr<TwoDofArm> right_arm = std::make_unique<TwoDofArm>(TwoDofArm());
};
