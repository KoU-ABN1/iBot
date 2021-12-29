#include "chassis.h"

void Chassis::moveToTable(const float vel, const float acc)
{
    std::vector<Eigen::Vector2f> nodes;
    nodes.push_back(Eigen::Vector2f(data.robot_x, data.robot_y));
    nodes.push_back(Eigen::Vector2f(4, 0));
    nodes.push_back(Eigen::Vector2f(4, -1));
    nodes.push_back(Eigen::Vector2f(1.5, -1));
    nodes.push_back(Eigen::Vector2f(1.5, -6));

    std::vector<Eigen::Vector2f> waypoints = generateWaypoints(nodes);
    drawWaypoints(waypoints);

    float track_dis = 0.2;
    static int index = 0;
    float dist = sqrt(pow(waypoints[index][0] - data.robot_x, 2) + pow(waypoints[index][1] - data.robot_y, 2));
    while (dist < track_dis && index < waypoints.size())
    {
        index++;
        dist = sqrt(pow(waypoints[index][0] - data.robot_x, 2) + pow(waypoints[index][1] - data.robot_y, 2));
    }

    moveToPointWithArc(waypoints[index], vel, 10);
}

std::vector<Eigen::Vector2f> Chassis::generateWaypoints(const std::vector<Eigen::Vector2f> &nodes)
{
    float bias = 0.5;
    std::vector<Eigen::Vector2f> ctr_points(3 * nodes.size() - 4);
    for (int i = 0; i < nodes.size(); i++)
    {
        if (i == 0)
            ctr_points[0] = nodes[0];
        else if (i == nodes.size() - 1)
            ctr_points[3 * i - 2] = nodes[i];
        else
        {
            ctr_points[3 * i - 2] = nodes[i] + (nodes[i - 1] - nodes[i]) / (nodes[i - 1] - nodes[i]).norm() * bias;
            ctr_points[3 * i - 1] = nodes[i];
            ctr_points[3 * i] = nodes[i] + (nodes[i + 1] - nodes[i]) / (nodes[i + 1] - nodes[i]).norm() * bias;
        }
    }

    std::vector<Eigen::Vector2f> way_points;
    int num = 100;
    for (int i = 0; i < 2 * nodes.size() - 3; i++)
    {
        if (i % 2 == 0)
        {
            Eigen::Vector2f p1 = ctr_points[3 * i / 2];
            Eigen::Vector2f p2 = ctr_points[3 * i / 2 + 1];
            float t = 0;
            for (int j = 0; j < num; j++)
            {
                Eigen::Vector2f p = (1 - t) * p1 + t * p2;
                t += 1.0 / (num - 1);
                way_points.push_back(p);
            }
        }
        else
        {
            Eigen::Vector2f p1 = ctr_points[(3 * i - 1) / 2];
            Eigen::Vector2f p2 = ctr_points[(3 * i - 1) / 2 + 1];
            Eigen::Vector2f p3 = ctr_points[(3 * i - 1) / 2 + 2];
            float t = 0;
            for (int j = 0; j < num; j++)
            {
                Eigen::Vector2f p = (1 - t) * (1 - t) * p1 + 2 * t * (1 - t) * p2 + t * t * p3;
                t += 1.0 / (num - 1);
                way_points.push_back(p);
            }
        }
    }

    return way_points;
}

void Chassis::drawWaypoints(const std::vector<Eigen::Vector2f> &waypoints)
{
    static bool flag = true;
    if (flag)
    {
        for (int i = 0; i < waypoints.size(); i++)
        {
            float point[3] = {waypoints[i][0], waypoints[i][1], 0};
            simAddDrawingObjectItem(handles.drawer, point);
        }
        flag = false;
    }
}

void Chassis::moveToCustomer(const float vel, const float acc)
{
    float bias = 1;
    Eigen::Vector2f p1(data.robot_x, data.robot_y);
    Eigen::Vector2f p2(data.robot_x + bias * cos(data.robot_yaw), data.robot_y + bias * sin(data.robot_yaw));
    Eigen::Vector2f p3(data.customer_x + bias * cos(data.customer_yaw), data.customer_y + bias * sin(data.customer_yaw));
    Eigen::Vector2f p4(data.customer_x, data.customer_y);

    float t = 0.1;
    Eigen::Vector2f target = p1 * (1 - t) * (1 - t) * (1 - t) + p2 * 3 * (1 - t) * (1 - t) * t + p3 * 3 * (1 - t) * t * t + p4 * t * t * t;

    float m = target[0] - data.robot_x;
    float n = target[1] - data.robot_y;
    float r = (m * m + n * n) / (2 * m * sin(data.robot_yaw) - 2 * n * cos(data.robot_yaw));
    float v1 = (r + TRACK_WIDTH) / r * vel;
    float v2 = (r - TRACK_WIDTH) / r * vel;

    left_wheel->setTargetVelocity(v1, acc);
    right_wheel->setTargetVelocity(v2, acc);
}

void Chassis::moveToPointWithArc(const Eigen::Vector2f &target, const float vel, const float acc)
{
    float m = target[0] - data.robot_x;
    float n = target[1] - data.robot_y;
    float r = (m * m + n * n) / (2 * m * sin(data.robot_yaw) - 2 * n * cos(data.robot_yaw));
    float v1 = (r + TRACK_WIDTH) / r * vel;
    float v2 = (r - TRACK_WIDTH) / r * vel;

    left_wheel->setTargetVelocity(v1, acc);
    right_wheel->setTargetVelocity(v2, acc);
}

void Chassis::stop(const float acc)
{
    left_wheel->setTargetVelocity(0, acc);
    right_wheel->setTargetVelocity(0, acc);
}

void Chassis::rotateInPlace(const float vel, const float acc)
{
    left_wheel->setTargetVelocity(-vel, acc);
    right_wheel->setTargetVelocity(vel, acc);
}
