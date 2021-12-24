#include "actions.h"

void Robot::waitAtDoor()
{
    chassis->stop();
}

void Robot::moveToCustomer()
{
    const float STOP_DIS = 1.2;

    float dist = sqrt(pow(robot.x - customer.x, 2) + pow(robot.y - customer.y, 2));

    if (dist > STOP_DIS)
    {
        Eigen::Vector2f target = chassis->BezierPlanner();
        chassis->moveToPointWithArc(target);

        body->trackCustomerFace();
    }
    else
    {
        chassis->stop();

        body->trackCustomerFace();
    }
}

void Robot::interactWithCustomerAtDoor()
{
}

void Robot::getTableNumber()
{
}

void Robot::takeCustomerToTable()
{
}

void Robot::interactWithCustomerAtTable()
{
}

void Robot::backToDoor()
{
}