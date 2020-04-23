//
// Created by Maciej Czechowski on 19/04/2020.
//

#ifndef PATH_PLANNING_CAR_H
#define PATH_PLANNING_CAR_H
#include "domain.h"

class Car {
public:
    Car() : desired_speed(49.8), x(0), y(0), s(0), d(0), yaw(0), speed(0) {}
    double x;
    double y;
    double s;
    double d;
    double yaw;
    double speed;
    double desired_speed;


};


#endif //PATH_PLANNING_CAR_H
