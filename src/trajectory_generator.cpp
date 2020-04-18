//
// Created by Maciej Czechowski on 18/04/2020.
//


#include "trajectory_generator.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "spline.h"
#include <iostream>

std::vector<std::vector<double>> TrajectoryGenerator::getNextPoints(int lane,
        double car_s,
        double car_x,
        double car_y,
        double car_yaw,
        double car_speed,
        double end_path_s,
        std::vector<double> previous_path_x,
        std::vector<double> previous_path_y,
        std::vector<std::vector<double>> sensor_fusion

        ) {

    int prev_size = previous_path_x.size();

    if (prev_size > 0)
    {
        car_s = end_path_s;
    }

    bool too_close = false;
    for (auto & other_car : sensor_fusion){
        //car in my lane?
        float d= other_car[6];
        if (d < (2+4*lane+2) && d > (2+4*lane-2)){
            //      std::cout << " other d is " << d << " and it is in my lane! my d is " << car_d << std::endl;
            //speed of the other car
            double  vx = other_car[3];
            double  vy = other_car[4];
            double check_speed = sqrt(vx*vx + vy*vy);
            double check_car_s = other_car[5];

            check_car_s += ((double)prev_size*0.02*check_speed); //where if will be?

            // std::cout << "the car will be at s " << check_car_s << " while my is " << car_s << " which makes gap of " << check_car_s-car_s << std::endl;
            //car in front and gap too small
            if ( (check_car_s > car_s) && ((check_car_s-car_s) < 30) ){
                too_close = true;
                //todo: this is for behavior planning - change lane if possible and worth it

                if (lane > 0){
                    lane = 0;
                }
            }

        }

    }

    if (too_close){
        ref_velocity -= .224; //~5m/s
    } else if (ref_velocity < 49.5){

        ref_velocity += .224;
    }

    if (ref_velocity < 0){
        ref_velocity = 0;
    }
       std::cout << "too close " << too_close << std::endl;
       std::cout << "reference velocity " << ref_velocity << std::endl;
    //a (spare) list of waypoints - spaced evenly at 30m
    //serves as an input for spline fit
    std::vector<double> ptsx;
    std::vector<double> ptsy;

    //reference x, y, yaw states
    // either we will reference the starting pount as where the car is or at previous path end
    double ref_x = car_x;
    double ref_y = car_y;
    double ref_yaw = deg2rad(car_yaw);

    //almost empty path - use car as starting reference
    if (prev_size < 2) {
        //tangent path
        double prev_car_x = car_x - cos(car_yaw);
        double prev_car_y = car_y - sin(car_yaw);

        ptsx.push_back(prev_car_x);
        ptsx.push_back(car_x);

        ptsy.push_back(prev_car_y);
        ptsy.push_back(car_y);
    } else {
        // new reference state - previous path end
        ref_x = previous_path_x[prev_size - 1];
        ref_y = previous_path_y[prev_size - 1];

        double ref_x_prev = previous_path_x[prev_size - 2];
        double ref_y_prev = previous_path_y[prev_size - 2];
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

        // two points that make path tangent to the previous end
        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);

        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);
    }

    vector<double> next_wp0 = getXY(car_s + 30, (2 + 4 * lane), map_waypoints_s, map_waypoints_x,
                                    map_waypoints_y);
    vector<double> next_wp1 = getXY(car_s + 60, (2 + 4 * lane), map_waypoints_s, map_waypoints_x,
                                    map_waypoints_y);
    vector<double> next_wp2 = getXY(car_s + 90, (2 + 4 * lane), map_waypoints_s, map_waypoints_x,
                                    map_waypoints_y);

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
    for (int i = 0; i < previous_path_x.size(); i++) {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }

    // calculate how to break up spline points so we travel at desired velocity
    double target_x = 30.0;
    double target_y = s(target_x);
    double target_dist = sqrt(target_x * target_x + target_y * target_y);

    double x_add_on = 0;
//fill up the path up to 50 pts
    for (int i = 0; i <= 50 - previous_path_x.size(); i++) {
        const double TICK = 0.02;
        const double VELOCITY_MOD = 2.24; //mph to m/s

        double N = (target_dist /
                    (TICK * ref_velocity / VELOCITY_MOD)); //number of points to match velocity
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
    std::vector<std::vector<double>> result { next_x_vals, next_y_vals};
    return result;
}
