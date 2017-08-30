//
// Created by Oleg Leyzerov on 23/08/2017.
//

#include "Track.h"
#include <vector>
#include "spline.h"

Track::Track(vector<double> map_waypoints_x,
             vector<double> map_waypoints_y,
             vector<double> map_waypoints_s,
             vector<double> map_waypoints_dx,
             vector<double> map_waypoints_dy) {

  x_s.set_points(map_waypoints_s, map_waypoints_x);
  y_s.set_points(map_waypoints_s, map_waypoints_y);
  dx_s.set_points(map_waypoints_s, map_waypoints_dx);
  dy_s.set_points(map_waypoints_s, map_waypoints_dy);

}

vector<double> Track::getXY(double s,double d) {
  double x = x_s(s) + dx_s(s) * d;
  double y = y_s(s) + dy_s(s) * d;
  return {x,y};
}

void Track::Init(vector<double> map_waypoints_x,
                 vector<double> map_waypoints_y,
                 vector<double> map_waypoints_s,
                 vector<double> map_waypoints_dx,
                 vector<double> map_waypoints_dy) {
  map_x = map_waypoints_x;
  map_y = map_waypoints_y;

  x_s.set_points(map_waypoints_s, map_waypoints_x);
  y_s.set_points(map_waypoints_s, map_waypoints_y);
  dx_s.set_points(map_waypoints_s, map_waypoints_dx);
  dy_s.set_points(map_waypoints_s, map_waypoints_dy);
}