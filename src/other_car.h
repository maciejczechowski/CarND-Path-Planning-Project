//
// Created by Maciej Czechowski on 20/04/2020.
//

#ifndef PATH_PLANNING_OTHER_CAR_H
#define PATH_PLANNING_OTHER_CAR_H

#include <deque>
using std::deque;

struct OtherCar {
    int id;
    double curr_x;
    double curr_y;
    double curr_vx;
    double curr_vy;
    double curr_s;
    double curr_d;
    deque<double> x;
    deque<double> y;
    deque<double> vx;
    deque<double> vy;
    deque<double> s;
    deque<double> d;

};


#endif //PATH_PLANNING_OTHER_CAR_H
