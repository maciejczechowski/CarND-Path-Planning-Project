//
// Created by Maciej Czechowski on 19/04/2020.
//

#ifndef PATH_PLANNING_SENSOR_FUSION_H
#define PATH_PLANNING_SENSOR_FUSION_H
#include <deque>
#include <vector>
#include <map>
#include "car.h"
#include "predictions.h"
using std::deque;
using std::vector;



class SensorFusion {
public:
    explicit SensorFusion(Predictions &predictionModule) : predictions(predictionModule) {};
    void setFusionData(vector<vector<double>> &fusionData);
    //returns data in form {velocity, distance} of the nearest car in the given lane
    vector<double> getVelocityAndDistanceToNearestInLane(int current_lane, int target_lane, Car &car);

    vector<OtherCar*> getAheadAndBehind(int lane, Car &car);
    vector<OtherCar*> getAheadAndBehind(Car &car);

private:
    Predictions& predictions;
    vector<vector<double>> fusion_data;
    std::map<int, OtherCar*> fusion_map;
};


#endif //PATH_PLANNING_SENSOR_FUSION_H
