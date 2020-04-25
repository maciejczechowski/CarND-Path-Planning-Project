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
    std::vector<double> s_values;
    std::vector<double> d_values;
    std::vector<double> theta;
    int finalLane;
    double velocity;
    double end_path_s;
    State forState;

    std::vector<double> GetBBox() {
        double minx = 99999, maxx = 0, miny = 99999, maxy = 0;
        for (int i = 0; i < x_values.size(); i++) {
            if (x_values[i] <= minx) {
                minx = x_values[i];
            }
            if (x_values[i] > maxx) {
                maxx = x_values[i];
            }
            if (y_values[i] <= miny) {
                miny = y_values[i];
            }
            if (y_values[i] < miny) {
                miny = x_values[i];
            }
        }
        return {minx, miny, maxx, maxy};
    }
};

struct OtherCar {
    int id;
    double curr_x;
    double curr_y;
    double curr_vx;
    double curr_vy;
    double curr_s;
    double curr_d;
    double curr_ax;
    double curr_ay;
    deque<double> x;
    deque<double> y;
    deque<double> vx;
    deque<double> vy;
    deque<double> s;
    deque<double> d;
    Trajectory *predictedTrajectory;

};


#endif //PATH_PLANNING_DOMAIN_H
