//
// Created by Maciej Czechowski on 18/04/2020.
//


#ifndef PATH_PLANNING_TRAJECTORY_GENERATOR_H
#define PATH_PLANNING_TRAJECTORY_GENERATOR_H
#include <vector>
#include "map.h"
#include "car.h"
#include "domain.h"

class TrajectoryGenerator {
public:
    explicit TrajectoryGenerator(Map &track_map ) : map(track_map) {};

    Trajectory getTrajectory(int lane, double targetVelocity, Car &car, Trajectory &previous_path);

private:
    Map map;

};


#endif //PATH_PLANNING_TRAJECTORY_GENERATOR_H
