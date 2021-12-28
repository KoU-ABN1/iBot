#include "differential_chassis.h"

void DifferentialChassis::moveToTable(const float vel_set)
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

    moveToPointWithArc(waypoints[index], vel_set, 10);

    static bool flag2 = true;
    if (flag2)
    {
        float d = atan2(nodes[1][1] - data.robot_y, nodes[1][0] - data.robot_x);
        if (abs(d - data.robot_yaw) > 0.1)
        {
            simSetJointTargetVelocity(handles.left_wheel, -3);
            simSetJointTargetVelocity(handles.right_wheel, 3);
        }
        else
        {
            stop(1);
            flag2 = false;
        }
    }
}

std::vector<Eigen::Vector2f> DifferentialChassis::generateWaypoints(const std::vector<Eigen::Vector2f> &nodes)
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

void DifferentialChassis::drawWaypoints(const std::vector<Eigen::Vector2f> &waypoints)
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

void DifferentialChassis::moveToCustomer(const float vel_set)
{
    const float bias = 1;
    Eigen::Vector2f p1(data.robot_x, data.robot_y);
    Eigen::Vector2f p2(data.robot_x + bias * cos(data.robot_yaw), data.robot_y + bias * sin(data.robot_yaw));
    Eigen::Vector2f p3(data.customer_x + bias * cos(data.customer_yaw), data.customer_y + bias * sin(data.customer_yaw));
    Eigen::Vector2f p4(data.customer_x, data.customer_y);

    float t = 0.1;
    Eigen::Vector2f target = p1 * (1 - t) * (1 - t) * (1 - t) + p2 * 3 * (1 - t) * (1 - t) * t + p3 * 3 * (1 - t) * t * t + p4 * t * t * t;

    float m = target[0] - data.robot_x;
    float n = target[1] - data.robot_y;
    float r = (m * m + n * n) / (2 * m * sin(data.robot_yaw) - 2 * n * cos(data.robot_yaw));
    float v1 = (r + D) / r * vel_set;
    float v2 = (r - D) / r * vel_set;

    left_wheel->setVelocity(v1);
    right_wheel->setVelocity(v2);
}

void DifferentialChassis::moveToPointWithArc(const Eigen::Vector2f &target, const float vel_set, const float acc_max)
{
    float m = target[0] - data.robot_x;
    float n = target[1] - data.robot_y;
    float r = (m * m + n * n) / (2 * m * sin(data.robot_yaw) - 2 * n * cos(data.robot_yaw));
    float v1 = (r + D) / r * vel_set;
    float v2 = (r - D) / r * vel_set;

    left_wheel->setAccMax(acc_max);
    right_wheel->setAccMax(acc_max);

    left_wheel->setVelocity(v1);
    right_wheel->setVelocity(v2);
}

void DifferentialChassis::stop(const float acc_max)
{
    left_wheel->setAccMax(acc_max);
    right_wheel->setAccMax(acc_max);

    left_wheel->setVelocity(0);
    right_wheel->setVelocity(0);
}

void DifferentialChassis::rotateCounterclockwise()
{
    float move_acc = 10;
    float vel_set = 0.5;

    left_wheel->setAccMax(move_acc);
    right_wheel->setAccMax(move_acc);

    left_wheel->setVelocity(-vel_set);
    right_wheel->setVelocity(vel_set);
}

void DifferentialChassis::rotateClockwise()
{
    float move_acc = 10;
    float vel_set = 0.5;

    left_wheel->setAccMax(move_acc);
    right_wheel->setAccMax(move_acc);

    left_wheel->setVelocity(vel_set);
    right_wheel->setVelocity(-vel_set);
}