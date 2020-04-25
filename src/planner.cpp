//
// Created by Maciej Czechowski on 20/04/2020.
//

#include "planner.h"
#include "helpers.h"
#include <iostream>
#include <string>

std::string StateNames[3] =
        {
                "FOLLOW",
                "CHANGE_LEFT",
                "CHANGE_RIGHT"
        };


Trajectory Planner::Execute(Trajectory &previousPath) {

//check for collisions

    int currentLane = Helper::getLane(car.d);
    if (Helper::isFullyOnLane(car.d)) {
        if (currentLane != this->current_lane) {
            std::cout << " Fully chaned lane to: " << currentLane << std::endl;
            this->current_lane = currentLane;
        }


    }

    auto trajectories = CalculateOptions(previousPath);


    Trajectory winning = trajectories[0];
    double winningCost = 99999999;

    std::string costDisplay = "costs: ";
   // std::cout << "costs ";
    for (auto &t: trajectories) {
        double costValue = cost.calculateCost(t);
        costDisplay += "\t" + StateNames[t.forState]+ ": "+ std::to_string(costValue);


      //  std::cout << "\t" << t.forState << ": " <<costValue;
        if (costValue < winningCost) {
            winningCost = costValue;
            winning = t;
        }
    }
    std::cout << costDisplay << std::endl;

   if (current_state != winning.forState){

       std::cout << costDisplay << std::endl;
       std::cout << "Swich to state: " << StateNames[winning.forState] << std::endl << std::endl;
   }
    desired_lane = winning.finalLane;
    current_state = winning.forState;
    ref_velocity = winning.velocity;
//std::cout << "new states" << winning.x_values.size() << std::endl;
    return winning;
}

vector<Trajectory> Planner::CalculateOptions(Trajectory &previousPath)
{
    vector<Trajectory> trajectory_options;

    switch (current_state) {
        case follow_lane:

            trajectory_options.push_back(GetTrajectory(previousPath, current_lane, follow_lane));

            if (current_lane > 0){
                trajectory_options.push_back(GetTrajectory(previousPath, current_lane - 1, change_left));
            }

            if (current_lane < 2){
                trajectory_options.push_back(GetTrajectory(previousPath, current_lane + 1, change_right));
            }

            break;
        case change_left:
                if (current_lane == desired_lane){  //finished lane change - switch to follow
                trajectory_options.push_back( GetTrajectory(previousPath, current_lane, follow_lane));

            } else { //not finished
                trajectory_options.push_back( GetTrajectory(previousPath, desired_lane, change_left)); //continue maneuver
            //   trajectory_options.push_back( GetTrajectory(previousPath, current_lane, change_right)); //abort maneuver
            }
            break;

        case change_right:

            if (current_lane == desired_lane){  //finished lane change - switch to follow
                trajectory_options.push_back( GetTrajectory(previousPath, current_lane, follow_lane));

            } else { //not finished
                trajectory_options.push_back( GetTrajectory(previousPath, desired_lane, change_right)); //continue maneuver
              // trajectory_options.push_back( GetTrajectory(previousPath, current_lane, change_left)); //abort maneuver
            }

    }
    return trajectory_options;
}


Trajectory Planner::GetTrajectory(Trajectory &previousPath, int forLane, State forState) {
    double velocity=UpdateDesiredVelocity();
    auto trajectory = trajectoryGenerator.getTrajectory(forLane, velocity, car, previousPath);
    trajectory.finalLane = forLane;
    trajectory.forState = forState;
    trajectory.velocity = velocity;
    return trajectory;
}

//Trajectory Planner::ChangeLaneLeft(Trajectory &previousPath, int desiredLane) {
//
//    auto trajectory = trajectoryGenerator.getTrajectory(desiredLane, ref_velocity, car, previousPath);
//    trajectory.finalLane = desiredLane;
//    trajectory.forState = change_left;
//    return trajectory;
//}
//
//Trajectory Planner::ChangeLaneRight(Trajectory &previousPath, int desiredLane) {
//
//    auto trajectory = trajectoryGenerator.getTrajectory(desiredLane, ref_velocity, car, previousPath);
//    trajectory.finalLane = desiredLane;
//    return trajectory;
//}

double Planner::UpdateDesiredVelocity() {
    auto followedCar = sensorFusion.getVelocityAndDistanceToNearestInLane(current_lane, desired_lane, car);
    double distanceToOther = followedCar[1];
    double speedOfOther = followedCar[0];
    double speedOfUs = Helper::mph2mps(car.speed);

    double next_velocity = ref_velocity;
    bool following = false;
    //way too close - brake!
    if (distanceToOther < 10) {
        following = true;
        next_velocity -= Helper::DesiredVelocityChange * 1.5;
        std::cout << "BRAKE!" << std::endl;
    } else
        if (distanceToOther < 40) {
        following = true;
        auto ds = Helper::DesiredVelocityChange ;
        if (speedOfUs > speedOfOther) {

          //  std::cout << "Lowering velocity " <<  std::min(ds, speedOfUs - speedOfOther)<< std::endl;
            next_velocity -= std::min(ds, speedOfUs - speedOfOther);
        } else {
          //  std::cout << "Raising velocity our/ther" << std::max(ds, speedOfOther-speedOfUs) << std::endl;
            next_velocity += std::min(ds, speedOfOther-speedOfUs);
        }
    }

    if (!following && next_velocity < car.desired_speed) {
        next_velocity += Helper::DesiredVelocityChange;
    }

    if (next_velocity < 0) {
        next_velocity = 0;
    } else if (next_velocity > car.desired_speed){
        next_velocity = car.desired_speed;
    }

    return next_velocity;
}