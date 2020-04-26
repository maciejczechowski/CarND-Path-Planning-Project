//
// Created by Maciej Czechowski on 20/04/2020.
//

#ifndef PATH_PLANNING_DOMAIN_H
#define PATH_PLANNING_DOMAIN_H

#include <deque>
#include <vector>
#include <string>

using std::deque;


enum State {
    follow_lane,
    change_left,
    change_right
};


class Trajectory {
public:
    std::vector<double> x_values;
    std::vector<double> y_values;

    int finalLane;
    double velocity;
    double end_path_s;
    State forState;

};

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
    Trajectory *predictedTrajectory;
};


#endif //PATH_PLANNING_DOMAIN_H
