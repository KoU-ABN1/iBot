#pragma once
#include "sim_plugin.h"

struct Point
{
    float x;
    float y;
    Point() {}
    Point(float t1, float t2) : x(t1), y(t2) {}

    Point operator+(Point p)
    {
        return Point(p.x + x, p.y + y);
    }
    Point operator*(float k)
    {
        //return Point(k * this->x, k * this->y);
        Point p;
        p.x = k * x;
        p.y = k * y;
        return p;
    }
};

void mainSimulation();
void faceTracking();
void moveToCustomer();