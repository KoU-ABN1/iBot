#include "sim_main.h"
#include "common.h"
#include "actions.h"

void mainSimulation()
{
    updateAllInfo();

    static Robot robot;
    static int state = WAIT_AT_DOOR;

    switch (state)
    {
    case WAIT_AT_DOOR:
    {
        state = robot.waitAtDoor();
        break;
    }

    case MOVE_TO_CUSTOMER:
    {
        state = robot.moveToCustomer();
        break;
    }
        state = robot.moveToCustomer();
        break;

    case INTERACT_WITH_CUSTOMER_AT_DOOR:
    {
        state = robot.interactWithCustomerAtDoor();
        break;
    }

    case GET_TABLE_NUMBER:
    {
        state = robot.getTableNumber();
        break;
    }

    case TAKE_CUSTOMER_TO_TABLE:
    {
        state = robot.takeCustomerToTable();
        break;
    }

    case INTERACT_WITH_CUSTOMER_AT_TABLE:
    {
        state = robot.interactWithCustomerAtTable();
        break;
    }

    case BACK_TO_DOOR:
    {
        state = robot.backToDoor();
        break;
    }
    }
}