//
// Created by Maciej Czechowski on 20/04/2020.
//

#ifndef PATH_PLANNING_PLANNER_H
#define PATH_PLANNING_PLANNER_H

#include "trajectory_generator.h"
#include "sensor_fusion.h"

enum State {
    follow_lane
};

class Planner {

public:
    Planner(TrajectoryGenerator &traj, SensorFusion &fusion, Car &autoCar)
            : trajectoryGenerator(traj),
              sensorFusion(fusion),
              car(autoCar),
              current_lane(1) {}

    Trajectory Execute(Trajectory &previousPath);

private:
    State current_state;
    TrajectoryGenerator &trajectoryGenerator;
    SensorFusion &sensorFusion;

    int current_lane;
    Car &car;

    Trajectory FollowTheLane(Trajectory &previousPath);

    double ref_velocity;

    void UpdateDesiredVelocity();

};


#endif //PATH_PLANNING_PLANNER_H
