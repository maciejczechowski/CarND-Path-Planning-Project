//
// Created by Maciej Czechowski on 18/04/2020.
//

#ifndef PATH_PLANNING_MAP_H
#define PATH_PLANNING_MAP_H
#include <vector>
#include <string>
using std::vector;
using std::string;

class Map {
public:
    void load_map(string filename);
    vector<double> getFrenet(double x, double y, double theta);
    vector<double> getXY(double s, double d);

private:
    // Returns next waypoint of the closest waypoint
    int NextWaypoint(double x, double y, double theta);
    int ClosestWaypoint(double x, double y);
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;
};


#endif //PATH_PLANNING_MAP_H
