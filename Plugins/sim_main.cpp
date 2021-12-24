#include "sim_main.h"
#include "motor.h"

void mainSimulation()
{
    updateAllInfo();

    static DifferentialChassis chassis;
    static ThreePartBody body;

    Point target = chassis.BezierPlanner();
    chassis.moveToPoint(target);

    body.trackCustomerFace();
}