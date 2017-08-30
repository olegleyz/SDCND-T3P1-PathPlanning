//
// Created by Oleg Leyzerov on 23/08/2017.
//

#ifndef PATH_PLANNING_TRACK_H
#define PATH_PLANNING_TRACK_H

#include <vector>
#include "spline.h"
using namespace std;

class Track {
public:
    vector<double> map_x;
    vector<double> map_y;
    tk::spline x_s;
    tk::spline y_s;
    tk::spline dx_s;
    tk::spline dy_s;
    Track(vector<double> map_waypoints_x,
          vector<double> map_waypoints_y,
          vector<double> map_waypoints_s,
          vector<double> map_waypoints_dx,
          vector<double> map_waypoints_dy);
    Track() {}
    ~Track() {}
    vector<double> getXY(double s,double d);
    void Init(vector<double> map_waypoints_x,
              vector<double> map_waypoints_y,
              vector<double> map_waypoints_s,
              vector<double> map_waypoints_dx,
              vector<double> map_waypoints_dy);
};


#endif //PATH_PLANNING_TRACK_H
