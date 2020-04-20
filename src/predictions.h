//
// Created by Maciej Czechowski on 19/04/2020.
//

#ifndef PATH_PLANNING_PREDICTIONS_H
#define PATH_PLANNING_PREDICTIONS_H

#include <vector>
#include "other_car.h"

using std::vector;

class Predictions {
public:
    //predict the other car s,d coordinates in given stepsNum
    vector<double> predictSD(OtherCar car, int stepsNum);
};


#endif //PATH_PLANNING_PREDICTIONS_H
