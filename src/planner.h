//
// Created by Maciej Czechowski on 20/04/2020.
//

#ifndef PATH_PLANNING_PLANNER_H
#define PATH_PLANNING_PLANNER_H

#include "trajectory_generator.h"
#include "sensor_fusion.h"
#include "cost.h"

class Planner {

public:
    Planner(TrajectoryGenerator &traj, SensorFusion &fusion, Car &autoCar, Cost &cost)
            : trajectoryGenerator(traj),
              sensorFusion(fusion),
              car(autoCar),
              current_lane(1),
              cost(cost),
              current_state(follow_lane){}

    Trajectory Execute(Trajectory &previousPath);

private:
    State current_state;
    TrajectoryGenerator &trajectoryGenerator;
    SensorFusion &sensorFusion;
    Cost& cost;

    int current_lane;
    int desired_lane;
    Car &car;

    double ref_velocity;

    double UpdateDesiredVelocity();
    Trajectory GetTrajectory(Trajectory &previousPath, int forLane, State forState);

    vector<Trajectory> CalculateOptions(Trajectory &previousPath);
};


#endif //PATH_PLANNING_PLANNER_H
