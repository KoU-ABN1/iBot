#include <math.h>
#include <iostream>
#include <vector>

#define PI 3.14159265
#define DEG 180.0 / 3.14159265

using namespace std;

int main()
{
    float m = 0;
    float n = -10;
    float p = 0;

    vector<pair<float, float>> result;

    float theta1 = atan2(m, p);
    float theta2 = atan2(-n * cos(theta1), p);
    result.push_back(make_pair(theta1, theta2));

    if (theta2 < 0)
        result.push_back(make_pair(theta1, theta2 + PI));
    else
        result.push_back(make_pair(theta1, theta2 - PI));

    if (theta1 < 0)
    {
        theta1 += PI;
        theta2 = atan2(-n * cos(theta1), p);
        result.push_back(make_pair(theta1, theta2));

        if (theta2 < 0)
            result.push_back(make_pair(theta1, theta2 + PI));
        else
            result.push_back(make_pair(theta1, theta2 - PI));
    }
    else
    {
        theta1 -= PI;
        theta2 = atan2(-n * cos(theta1), p);
        result.push_back(make_pair(theta1, theta2));

        if (theta2 < 0)
            result.push_back(make_pair(theta1, theta2 + PI));
        else
            result.push_back(make_pair(theta1, theta2 - PI));
    }

    for (int i = 0; i < result.size(); i++)
    {
        cout << result[i].first * DEG << "    " << result[i].second * DEG << endl;
        float r = -sin(result[i].first) * cos(result[i].second);
        float s = sin(result[i].second);
        float t = -cos(result[i].first) * cos(result[i].second);
        cout << r << endl;
        cout << s << endl;
        cout << t << endl;
        cout << endl;
    }

    return 0;
}