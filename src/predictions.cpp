//
// Created by Maciej Czechowski on 19/04/2020.
//

#include <cmath>
#include "predictions.h"
#include "helpers.h"
Trajectory* Predictions::predictTrajectory(OtherCar &car, int stepsNum) {

    double v0x = car.curr_vx;
    double v0y = car.curr_vy;

    double heading = 0;

    if (car.x.size() > 1) {
      heading = atan2((car.curr_x - car.x[car.x.size()-2]), (car.curr_y - car.y[car.y.size()-2]));
    }

    vector<double> next_x;
    vector<double> next_y;
    vector<double> next_s;
    vector<double> next_d;
    vector<double> next_heading;


    for (int tick=0; tick<stepsNum; tick++)
    {
        auto t = tick * Helper::TICK;
        auto x = car.curr_x + car.curr_vx * t;
        auto y = car.curr_y + car.curr_vy * t;

        next_x.push_back(x);
        next_y.push_back(y);
        next_heading.push_back(heading);
    }

    auto tr = new Trajectory();
    tr->finalLane = -1;
    tr->x_values= next_x;
    tr->y_values = next_y;
    return  tr;
}
