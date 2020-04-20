//
// Created by Maciej Czechowski on 20/04/2020.
//

#include "planner.h"
#include "helpers.h"
#import <iostream>

Trajectory Planner::Execute(Trajectory &previousPath) {

//todo: check for flags
//check for collisions
    return FollowTheLane(previousPath);
}


Trajectory Planner::FollowTheLane(Trajectory &previousPath) {
    UpdateDesiredVelocity();
    auto trajectory = trajectoryGenerator.getTrajectory(current_lane, ref_velocity, car, previousPath);
    return trajectory;
}

void Planner::UpdateDesiredVelocity() {
    auto followedCar = sensorFusion.getVelocityAndDistanceToNearestInLane(current_lane, car);
    double distanceToOther = followedCar[1];
    double speedOfOther = followedCar[0];
    double speedOfUs = Helper::mph2mps(car.speed);

    bool following = false;
    //way too close - brake!
std::cout << "my distance to following vehicle is " << distanceToOther << std::endl;
    if (distanceToOther < 20) {
        following = true;
        ref_velocity -= Helper::DesiredVelocityChange * 1.5;
    } else if (distanceToOther < 40) {
        following = true;
        if (speedOfUs > speedOfOther) {
            ref_velocity -= .224 / 3;
        } else {
            ref_velocity += .224 / 3;
        }
    }

    if (!following && ref_velocity < 49.5) {
        ref_velocity += Helper::DesiredVelocityChange;
    }

    if (ref_velocity < 0) {
        ref_velocity = 0;
    }


}
