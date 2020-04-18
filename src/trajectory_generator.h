//
// Created by Maciej Czechowski on 18/04/2020.
//


#ifndef PATH_PLANNING_TRAJECTORY_GENERATOR_H
#define PATH_PLANNING_TRAJECTORY_GENERATOR_H
#include <vector>
#include "map.h"
#include "car.h"


class TrajectoryGenerator {
public:
    explicit TrajectoryGenerator(Map &track_map ) : map(track_map), ref_velocity(0) {};

    std::vector<std::vector<double>> getTrajectory(int lane, Car &car,
                                                   double end_path_s,
                                                   std::vector<double> previous_path_x, std::vector<double> previous_path_y,
                                                   std::vector<std::vector<double>> sensor_fusion);

private:
    Map map;
    double ref_velocity;



};


#endif //PATH_PLANNING_TRAJECTORY_GENERATOR_H
