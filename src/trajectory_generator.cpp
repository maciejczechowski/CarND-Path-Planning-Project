//
// Created by Maciej Czechowski on 18/04/2020.
//


#include "trajectory_generator.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "spline.h"
#include "helpers.h"
#include <iostream>

Trajectory TrajectoryGenerator::getTrajectory(int lane, double targetVelocity, Car &car, Trajectory &previous_path) {

    auto prev_size = previous_path.x_values.size();
    //a (spare) list of waypoints - spaced evenly at 30m
    //serves as an input for spline fit
    std::vector<double> ptsx;
    std::vector<double> ptsy;

    //reference x, y, yaw states
    // either we will reference the starting pount as where the car is or at previous path end
    double ref_x = car.x;
    double ref_y = car.y;
    double ref_yaw = Helper::deg2rad(car.yaw);

    //almost empty path - use car as starting reference
    if (previous_path.x_values.size() < 2) {
        //tangent path
        double prev_car_x = car.x - cos(car.yaw);
        double prev_car_y = car.y - sin(car.yaw);

        ptsx.push_back(prev_car_x);
        ptsx.push_back(car.x);

        ptsy.push_back(prev_car_y);
        ptsy.push_back(car.y);
    } else {
        // new reference state - previous path end
        ref_x = previous_path.x_values[prev_size - 1];
        ref_y = previous_path.y_values[prev_size - 1];

        double ref_x_prev = previous_path.x_values[prev_size - 2];
        double ref_y_prev = previous_path.y_values[prev_size - 2];
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

        // two points that make path tangent to the previous end
        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);

        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);
    }

    vector<double> next_wp0 = map.getXY(car.s + 30, (2 + 4 * lane));
    vector<double> next_wp1 = map.getXY(car.s + 60, (2 + 4 * lane));
    vector<double> next_wp2 = map.getXY(car.s + 90, (2 + 4 * lane));

    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);

    for (int i = 0; i < ptsx.size(); i++) {

        //shift vehicle reference angle to 0
        double shift_x = ptsx[i] - ref_x;
        double shift_y = ptsy[i] - ref_y;

        ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
        ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));

    }

    tk::spline s;
    s.set_points(ptsx, ptsy);

    vector<double> next_x_vals;
    vector<double> next_y_vals;

    //start with previous path
    for (int i = 0; i < prev_size; i++) {
        next_x_vals.push_back(previous_path.x_values[i]);
        next_y_vals.push_back(previous_path.y_values[i]);
    }

    // calculate how to break up spline points so we travel at desired velocity
    double target_x = 30.0;
    double target_y = s(target_x);
    double target_dist = sqrt(target_x * target_x + target_y * target_y);

    double x_add_on = 0;
//fill up the path up to 50 pts
    for (int i = 0; i <= 30 - prev_size; i++) {

        double N = (target_dist /
                    (Helper::TICK * Helper::mph2mps(targetVelocity))); //number of points to match velocity
        double x_point = x_add_on + target_x / N;
        double y_point = s(x_point);

        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        //rotate back to normal
        x_point = x_ref * cos(ref_yaw) - y_ref*sin(ref_yaw);
        y_point = x_ref * sin(ref_yaw) + y_ref*cos(ref_yaw);

        x_point += ref_x;
        y_point += ref_y;

        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);


    }
     return { next_x_vals, next_y_vals};
}
