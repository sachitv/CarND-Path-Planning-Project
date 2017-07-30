//
// Created by Sachit Vithaldas on 29/07/17.
//

#ifndef PATH_PLANNING_UTIL_H
#define PATH_PLANNING_UTIL_H

#include <vector>
#include <math.h>

using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }

inline double deg2rad( double x ) { return x * pi() / 180; }

inline double rad2deg( double x ) { return x * 180 / pi(); }

inline double distance( double x1, double y1, double x2, double y2 )
{
	return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

int ClosestWaypoint( double x, double y, vector<double> maps_x, vector<double> maps_y );

int NextWaypoint( double x, double y, double theta, vector<double> maps_x, vector<double> maps_y );

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet( double x, double y, double theta, vector<double> maps_x, vector<double> maps_y );

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY( double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y );

#endif //PATH_PLANNING_UTIL_H
