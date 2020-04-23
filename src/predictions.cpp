//
// Created by Maciej Czechowski on 19/04/2020.
//

#include <cmath>
#include "predictions.h"
#include "helpers.h"
Trajectory* Predictions::predictTrajectory(OtherCar &car, int stepsNum) {

    double v0x = car.curr_vx;
    double v0y = car.curr_vy;

    double heading = atan2((car.curr_x -car.x[car.x.size()]),
                           (car.curr_y - car.y[car.y.size()]));

    vector<double> next_x;
    vector<double> next_y;
    vector<double> next_heading;


    for (int tick=0; tick<stepsNum; tick++)
    {
        auto t = tick * Helper::TICK;
        auto x = car.curr_x + v0x*t + 0.5*car.curr_ax * t * t;
        auto y = car.curr_y + v0y*t + 0.5*car.curr_ay * t * t;

        next_x.push_back(x);
        next_y.push_back(y);
        next_heading.push_back(heading);
    }

    //todo: caluclate final lane
    auto tr = new Trajectory();
    tr->finalLane = -1;
    tr->x_values=next_x;
    tr->y_values =next_y;
    tr->theta = next_heading;
    return  tr;


}
