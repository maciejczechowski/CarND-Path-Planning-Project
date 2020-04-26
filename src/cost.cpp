//
// Created by Maciej Czechowski on 20/04/2020.
//

#include <iostream>
#include "cost.h"
#include "helpers.h"

double Cost::calculateCost(Trajectory &trajectory) {

    double cost = calculateEfficiencyCost(trajectory.finalLane)
                  + calculateCollisionCost(trajectory)
                  + calculateLaneChangeCost(trajectory.finalLane) * 0.075
                  + trajectory.penalty;

    return cost;

}

double Cost::calculateLaneChangeCost(int finalLane) {

    if (finalLane < 0 || finalLane > 2) {
        return 999999;
    }

    if (finalLane != Helper::getLane(car.d)) {
        if (finalLane == 1) {
            return 0.5;
        } else {
            return 1;
        }
    }


    return 0;
}

double Cost::calculateEfficiencyCost(int finalLane) {
    double laneSpeed = getLaneSpeed(finalLane);
    double cost = double(2.0 * car.desired_speed - laneSpeed) / car.desired_speed;
    return cost;
}


double Cost::calculateCollisionCost(Trajectory &trajectory) {
    int ourLane = Helper::getLane(car.d);

    double cost = 0;
    vector<OtherCar *> aheadAndBehind = sensorFusion.getAheadAndBehind(car);

    for (auto &otherCar: aheadAndBehind) {
        auto otherLane = Helper::getLane(otherCar->curr_d);

        if (otherLane != ourLane && otherLane != trajectory.finalLane )
        {
            continue;
        }

        auto current_distance = Helper::distance(otherCar->curr_x, otherCar->curr_y, car.x, car.y);
        if (current_distance < 20){
            if (Helper::getLane(car.d) ==trajectory.finalLane) {
                return 1000;
            } else {
                return 10000;
            }
        }

        for (int i = 0; i < trajectory.x_values.size()-1; i++) {
            auto otherX = otherCar->predictedTrajectory->x_values[i];
            auto otherY = otherCar->predictedTrajectory->y_values[i];
            auto ourX = trajectory.x_values[i];
            auto ourY = trajectory.y_values[i];

            auto dist = Helper::distance(ourX, ourY, otherX, otherY);
            if (dist < 20) {

                if (Helper::getLane(car.d) ==trajectory.finalLane) {

                    return 1000;
                } else {
                    return 10000;
                }
            }
        }
    }

    return cost;
}

double Cost::getLaneSpeed(int lane) {
    vector<OtherCar *> cars = sensorFusion.getAheadAndBehind(lane, car);

    OtherCar *ahead = cars[0];
    if (ahead == nullptr || ahead->curr_s - car.s > 100) { //if vehicle is far away, do not take into account
        return car.desired_speed;
    }
    double currentSpeed = sqrt(ahead->curr_vx * ahead->curr_vx + ahead->curr_vy * ahead->curr_vy);

    double finalSpeedX = ahead->curr_vx;
    double finalSpeedY = ahead->curr_vy;
    double finalSpeed = sqrt(finalSpeedX * finalSpeedX + finalSpeedY * finalSpeedY);

    double laneSpeed = finalSpeed < currentSpeed ? finalSpeed : currentSpeed;

    return laneSpeed;
}
