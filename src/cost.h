//
// Created by Maciej Czechowski on 20/04/2020.
//

#ifndef PATH_PLANNING_COST_H
#define PATH_PLANNING_COST_H


#include "trajectory_generator.h"
#include "sensor_fusion.h"


class Cost {

public:
    Cost(SensorFusion& sensor_fusion, Car& vehicle, Map& map) : sensorFusion(sensor_fusion), car(vehicle), map(map) {}
    double calculateCost(Trajectory &trajectory);
private:
    double getLaneSpeed(int lane);
    SensorFusion& sensorFusion;
    Car& car;
    Map& map;

    double calculateEfficiencyCost(int finalLane);
    double calculateCollisionCost(Trajectory &trajectory);
    double calculateLaneChangeCost(int finalLane);

};


#endif //PATH_PLANNING_COST_H
