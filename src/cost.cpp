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
                  + CalculateVelocityCost(trajectory) * 0;
    return cost;

}

double Cost::calculateLaneChangeCost(int finalLane) {

    if (finalLane < 0 || finalLane > 2) {
        return 999999;
    }

    if (finalLane != Helper::getLane(car.d))
        if (finalLane == 1) {
            return  0.5;
        } else {
            return 1;
        }


    return 0;
}

double Cost::calculateEfficiencyCost(int finalLane) {
    double laneSpeed = getLaneSpeed(finalLane);
    double cost = double(2.0 * car.desired_speed - laneSpeed) / car.desired_speed;
    //std::cout << "eff cost for final lane " << finalLane << " " << cost << std::endl;
    return cost;
}


double Cost::CalculateVelocityCost(Trajectory &trajectory) {

    double cost = car.desired_speed / (trajectory.velocity + 0.001);
    // std::cout << "velocity cost for final lane " << trajectory.finalLane << " " << cost << std::endl;
    return cost;
}

/*

double Cost::calculateCollisionCost(Trajectory &trajectory) {

  //  std::cout << std::endl << " Checking T: " << trajectory.finalLane << std::endl;
    int ourLane = Helper::getLane(car.d);

    double cost = 0;
    vector<OtherCar*> aheadAndBehind = sensorFusion.getAheadAndBehind(car);

    //box for our
    auto our_bbox = trajectory.GetBBox();
    auto m_left = our_bbox[0];
    auto m_top = our_bbox[1];
    auto m_right = our_bbox[2];
    auto m_bottom = our_bbox[3];

//    m_left -= Helper::SafeDistancePass; //left
//    m_top -= Helper::SafeDistancePass; //top
//    m_right += Helper::SafeDistancePass; //right
//    m_bottom += Helper::SafeDistancePass; //bottom



    for (auto &otherCar: aheadAndBehind) {
        int otherLane = Helper::getLane(otherCar->curr_d);
        if (ourLane == otherLane || otherLane == trajectory.finalLane) {
            auto other_bbox = otherCar->predictedTrajectory->GetBBox();

            auto o_left = other_bbox[0];
            auto o_top = other_bbox[1];
            auto o_right = our_bbox[2];
            auto o_bottom = our_bbox[3];

            //do we collide?
            if ((m_left < o_right) && (m_right > o_left) && (m_top > o_bottom) && (m_bottom < o_top)) {

                if (Helper::getLane(car.d) != Helper::getLane(otherCar->curr_d)) {
                    cost += 1000;
                }
//                } else {
//                    cost += 10000;
//                }
            }
        }
    }

    return cost;
}
*/


double Cost::calculateCollisionCost(Trajectory &trajectory) {

    //  std::cout << std::endl << " Checking T: " << trajectory.finalLane << std::endl;
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

        assert(trajectory.s_values.size() == otherCar->predictedTrajectory->s_values.size());
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
    double d_cent = Helper::getLaneCenter(lane);
    vector<OtherCar *> cars = sensorFusion.getAheadAndBehind(lane, car);

    OtherCar *ahead = cars[0];
    if (ahead == nullptr || ahead->curr_s - car.s > 100) { //if vehicle is far away, do not take into account
        return car.desired_speed;
    }
    double currentSpeed = sqrt(ahead->curr_vx * ahead->curr_vx + ahead->curr_vy * ahead->curr_vy);

    double finalSpeedX = ahead->curr_vx + ahead->curr_ax * Helper::LookForward * Helper::TICK;
    double finalSpeedY = ahead->curr_vy + ahead->curr_ay * Helper::LookForward * Helper::TICK;
    double finalSpeed = sqrt(finalSpeedX * finalSpeedX + finalSpeedY * finalSpeedY);

    double laneSpeed = finalSpeed < currentSpeed ? finalSpeed : currentSpeed;

    return laneSpeed;
}
