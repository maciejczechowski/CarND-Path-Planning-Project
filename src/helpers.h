#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>
#include <iostream>

// for convenience
using std::string;
using std::vector;


//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

class Helper {
public:
    constexpr static const double TICK = 0.02;
    constexpr static const double DesiredVelocityChange = .224;
    constexpr static const double s_wraps_at = 6945.554;

    constexpr static const int LookForward = 50; // 50 ticks = 1s

    static constexpr double pi() { return M_PI; }

    static double deg2rad(double x) { return x * pi() / 180; }

    static double rad2deg(double x) { return x * 180 / pi(); }

    static double mph2mps(double mph) { return mph / 2.24; }

    static double getLaneCenter(int lane) { return 2.0 + 4 * lane; }

// Calculate distance between two points
    static double distance(double x1, double y1, double x2, double y2) {
        return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    }

    static int getLane(double d) {
        if (d < 4) return 0;
        if (d < 8) return 1;
        if (d < 12) return 2;

        return -1;
    }

    static bool isFullyOnLane(double d) {

        if (d > 1.5 && d < 2.5) return true;
        if (d > 5.5 && d < 6.5) return true;
        if (d > 9.5 && d < 10.5) return true;

        return false;
    }
};


#endif  // HELPERS_H