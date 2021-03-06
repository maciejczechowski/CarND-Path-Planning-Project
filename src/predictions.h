//
// Created by Maciej Czechowski on 19/04/2020.
//

#ifndef PATH_PLANNING_PREDICTIONS_H
#define PATH_PLANNING_PREDICTIONS_H

#include <vector>
#include "domain.h"
#include "trajectory_generator.h"

using std::vector;

class Predictions {

public:
    explicit Predictions(Map& map) : map(map) {}
    //predict the other car s,d coordinates in given stepsNum
    Trajectory* predictTrajectory(OtherCar &car, int stepsNum);

private:
    Map& map;
};


#endif //PATH_PLANNING_PREDICTIONS_H
