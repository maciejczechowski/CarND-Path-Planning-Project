//
// Created by Maciej Czechowski on 19/04/2020.
//

#include "sensor_fusion.h"
#include "Eigen-3.3/Eigen/Core"
#include "domain.h"
#include "helpers.h"
#include <iostream>
#include <string>

vector<double> SensorFusion::getVelocityAndDistanceToNearestInLane(int current_lane, int target_lane, Car &car) {

    double current_max = 99999999;
    double velocity = 0;

    for (auto &other_car : fusion_map) {
        //car in my lane?

        double other_d = other_car.second->curr_d;

        //in the same lane?
        if ((other_d < (2.0 + 4.0 * current_lane + 2.0) && other_d > (2.0 + 4 * current_lane - 2.0))
        ||(other_d < (2.0 + 4.0 * target_lane + 2.0) && other_d > (2.0 + 4 * target_lane - 2.0))  ) {

            double other_car_s = other_car.second->curr_s;
            double other_car_d = other_car.second->curr_d;
            if (other_car_s < car.s) { //behind us, ignore
                continue;
            }

            //speed of the other car
            double vx = other_car.second->curr_vx;
            double vy = other_car.second->curr_vy;
            double other_speed = sqrt(vx * vx + vy * vy);

            double distance = sqrt(pow(other_car_s - car.s, 2) + pow(other_car_d - car.d, 2));

            if (distance < current_max) {
                current_max = distance;
                velocity = other_speed;
            }

        }

    }
    return {velocity, current_max};
}

void SensorFusion::setFusionData(vector<vector<double>> &fusionData) {
    const int max_history = Helper::LookForward * 2;
    this->fusion_data = fusionData;

    for (auto &fusionEntry : fusionData) {
        OtherCar *car;

        if (fusion_map.find(fusionEntry[0]) == fusion_map.end()) {
            car = new OtherCar();
            car->id = fusionEntry[0];
            fusion_map[fusionEntry[0]] = car;
        } else {
            car = fusion_map[fusionEntry[0]];
        }

        car->curr_x = fusionEntry[1];
        car->x.push_back(car->curr_x);

        if (car->x.size() > max_history) {
            car->x.pop_front();
        }

        car->curr_y = fusionEntry[2];
        car->y.push_back(car->curr_y);
        if (car->y.size() > max_history) {
            car->y.pop_front();
        }

        car->curr_vx = fusionEntry[3];
        car->vx.push_back(car->curr_vx);
        if (car->vx.size() > max_history) {
            car->vx.pop_front();
        }

        car->curr_vy = fusionEntry[4];
        car->vy.push_back(car->curr_vy);
        if (car->vy.size() > max_history) {
            car->vy.pop_front();
        }

        car->curr_s = fusionEntry[5];
        car->s.push_back(car->curr_s);
        if (car->s.size() > max_history) {
            car->s.pop_front();
        }

        car->curr_d = fusionEntry[6];
        car->d.push_back(car->curr_d);
        if (car->d.size() > max_history) {
            car->d.pop_front();
        }

        auto trajectory = predictions.predictTrajectory(*car, Helper::LookForward);
        delete car->predictedTrajectory;
        car->predictedTrajectory = trajectory;

    }
}

vector<OtherCar *> SensorFusion::getAheadAndBehind(int lane, int target_lane, Car &car) {
    OtherCar *otherAhead = nullptr;
    OtherCar *otherBehind = nullptr;
    double minDistAhead = 999999;
    double minDistBehind = 999999;

    for (auto &other_car : fusion_map) {
        double other_d = other_car.second->curr_d;
        auto otherLane = Helper::getLane(other_d);
        //in the desired lane?
        if ( otherLane == lane || otherLane == target_lane ){
            double other_car_s = other_car.second->curr_s;
            double distance = fabs(car.s - other_car_s);
            if (other_car_s > car.s && distance < minDistAhead) {
                minDistAhead = distance;
                otherAhead = other_car.second;
            } else if (distance < minDistBehind) {
                minDistBehind = distance;
                otherBehind = other_car.second;
            }
        }
    }
    return {otherAhead, otherBehind};
}

vector<OtherCar *> SensorFusion::getAheadAndBehind(Car &car) {

    vector<OtherCar *> cars;
    double aheadDistances[] = {999999, 999999, 999999};
    double behindDistances[] = {999999, 999999, 999999};

    int aheadIs[] = {-1, -1, -1};
    int behindIds[] = {-1, -1, -1,};
    for (auto &other_car : fusion_map) {
        double other_d = other_car.second->curr_d;

        int lane = Helper::getLane(other_d);
        double other_car_s = other_car.second->curr_s;
        double distance = fabs(car.s - other_car_s);


        if (distance > 200) continue;

        if (other_car_s > car.s && distance < aheadDistances[lane]) {
            aheadDistances[lane] = distance;
            aheadIs[lane] = other_car.second->id;
        } else if (distance < behindDistances[lane]) {
            behindDistances[lane] = distance;
             behindIds[lane] = other_car.second->id;
        }

    }
    std::string ahead = "AHEAD: ";
    std::string behind = "BEHIND: ";
    for (int i = 0; i < 3; i++) {
        if (aheadIs[i] != -1) {
            ahead += std::to_string(aheadIs[i]) + " ";
            cars.push_back(fusion_map[aheadIs[i]]);
        }
        if (behindIds[i] != -1) {
            behind += std::to_string(behindIds[i]) + " ";
            cars.push_back(fusion_map[behindIds[i]]);
        }
    }

   // std::cout << ahead << std::endl << behind << std::endl;
    return cars;
}
