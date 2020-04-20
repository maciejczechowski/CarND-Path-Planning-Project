//
// Created by Maciej Czechowski on 19/04/2020.
//

#include "sensor_fusion.h"
#include "Eigen-3.3/Eigen/Core"
#include "other_car.h"
#include <iostream>

vector<double> SensorFusion::getVelocityAndDistanceToNearestInLane(int lane, Car &car) {

    double current_max = 99999999;
    double velocity = 0;

    for (auto &other_car : fusion_map) {
        //car in my lane?

        double other_d = other_car.second -> curr_d;
 
        //in the same lane?
        if (other_d < (2.0 + 4.0 * lane + 2.0) && other_d > (2.0 + 4 * lane - 2.0)) {


            double other_car_s = other_car.second -> curr_s;
            double other_car_d = other_car.second -> curr_d;
            if (other_car_s < car.s) { //behind us, ignore
                continue;
            }

            //speed of the other car
            double vx = other_car.second -> curr_vx;
            double vy = other_car.second -> curr_vy;
            double other_speed = sqrt(vx * vx + vy * vy);


            double distance = sqrt(pow(other_car_s - car.s,2) + pow(other_car_d - car.d, 2));

            if (distance < current_max)
            {
                current_max = distance;
                velocity = other_speed;
            }

        }

    }
    return {velocity, current_max};
}

void SensorFusion::setFusionData(vector <vector<double>> &fusionData) {
    const int max_history = 50;
    this->fusion_data = fusionData;
    for (auto &fusionEntry : fusionData) {

        OtherCar* car;

        if (fusion_map.find(fusionEntry[0]) == fusion_map.end()) {
            car = new OtherCar();
            fusion_map[fusionEntry[0]] = car;
        } else {
            car = fusion_map[fusionEntry[0]];
        }


        car -> curr_x = fusionEntry[1];
        car -> x.push_back(car->curr_x);

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
        car->vy.push_back( car->curr_vy);
        if (car->vy.size() > max_history) {
            car->vy.pop_front();
        }

        car->curr_s = fusionEntry[5];
        car->s.push_back(car->curr_s);
        if (car->s.size() > max_history) {
            car->s.pop_front();
        }

        car->curr_d = fusionEntry[6];
        car->d.push_back( car->curr_d);
        if (car->d.size() > max_history) {
            car->d.pop_front();
        }


    }
}
