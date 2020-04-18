#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>

// for convenience
using std::string;
using std::vector;


//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

class Helper {
public:
    static constexpr double pi() { return M_PI; }
    static double deg2rad(double x) { return x * pi() / 180; }
    static double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
    static double distance(double x1, double y1, double x2, double y2) {
        return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
    }
};






#endif  // HELPERS_H