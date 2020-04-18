//
// Created by Maciej Czechowski on 18/04/2020.
//


#ifndef PATH_PLANNING_TRAJECTORY_GENERATOR_H
#define PATH_PLANNING_TRAJECTORY_GENERATOR_H
#include <vector>



class TrajectoryGenerator {
public:
    TrajectoryGenerator( std::vector<double> &m_waypoints_x,  std::vector<double> &m_waypoints_y,  std::vector<double> &m_waypoints_s)
    : map_waypoints_x(m_waypoints_x), map_waypoints_y(m_waypoints_y), map_waypoints_s(m_waypoints_s) {}

    std::vector<std::vector<double>> getNextPoints(int lane, double car_s, double car_x, double car_y, double car_yaw, double car_speed,
                  double end_path_s,
                  std::vector<double> previous_path_x, std::vector<double> previous_path_y,
                  std::vector<std::vector<double>> sensor_fusion);

private:
    double ref_velocity;
    std::vector<double> map_waypoints_x;
    std::vector<double> map_waypoints_y;
    std::vector<double> map_waypoints_s;



};


#endif //PATH_PLANNING_TRAJECTORY_GENERATOR_H
