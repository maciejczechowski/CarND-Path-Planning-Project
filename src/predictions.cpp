//
// Created by Maciej Czechowski on 19/04/2020.
//

#include <cmath>
#include "predictions.h"

vector<double> Predictions::predictSD(OtherCar car, int stepsNum) {
    //todo: extrapolate (from spline?)
    int backidx = car.vx.size()-1;

    double vx = car.vx[backidx];
    double vy = car.vy[backidx];
    double s = car.s[backidx];
    double d = car.d[backidx];
    double check_speed = sqrt(vx*vx + vy*vy);


    s += ((double)stepsNum*0.02*check_speed); //where if will be?

    return {s,d};
}
