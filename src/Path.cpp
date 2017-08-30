//
// Created by Oleg Leyzerov on 19/08/2017.
//

#include <cmath>
#include <vector>
#include <string>
#include <iostream>
#include "Path.h"

using namespace std;


Path::Path(vector<double> map_waypoints_x,
           vector<double> map_waypoints_y,
           vector<double> map_waypoints_s,
           vector<double> map_waypoints_dx,
           vector<double> map_waypoints_dy){

  behav_plan.Init(map_waypoints_x,
                  map_waypoints_y,
                  map_waypoints_s,
                  map_waypoints_dx,
                  map_waypoints_dy);
}



vector<vector<double>> Path::Process(double car_s,
                                     double car_d,
                                     vector<vector<double>> sensor_fusion,
                                     vector<double> previous_path_x,
                                     vector<double> previous_path_y) {



  behav_plan.predictions.update(behav_plan.track, sensor_fusion); // update info about vehicles around
  vector<vector<double>> next_path_XY_points = behav_plan.GetBestTrajectory(car_s,
                                                                    car_d,
                                                                    sensor_fusion,
                                                                    previous_path_x,
                                                                    previous_path_y);

  return next_path_XY_points;


}
//
// Behavior Planing Class functions
//

void BehavPlan::Init(vector<double> map_waypoints_x,
                     vector<double> map_waypoints_y,
                     vector<double> map_waypoints_s,
                     vector<double> map_waypoints_dx,
                     vector<double> map_waypoints_dy) {
  // init track splines
  track.Init(map_waypoints_x,
             map_waypoints_y,
             map_waypoints_s,
             map_waypoints_dx,
             map_waypoints_dy);
}

vector<vector<double>> BehavPlan::CarsAhead(double car_s, int lane, double behind, double length, bool collision){
  vector<vector<double>> cars_ahead;
  double buffer = 20; // distance to the car ahead in collision avoidance case
  for (auto i = 0; i < predictions.current_sf.size(); i++){
    double d_lane_center = lane * 4 + 2;
    double d_veh = predictions.current_sf[i][6];
    int veh_id = predictions.current_sf[i][0];
    double veh_ad = predictions.vehicles[veh_id].ad;

    bool cars_left = ((d_lane_center-d_veh)>=2.0 && (d_lane_center-d_veh)<2.4 && (veh_ad > 0.5));
    bool cars_right = ((d_lane_center-d_veh)<=-2.0 && (d_lane_center-d_veh)>-2.4 && (veh_ad < -0.5));
    bool collision_adjuscent = collision && (cars_left || cars_right);

    if (fabs(d_lane_center-d_veh)<2) { //vehicle in lane ahead
      double distance = predictions.current_sf[i][5] - (car_s+behind);
      if (distance < 0) distance = 6945.554 + distance;
      if (distance < length){ //veh ahead
        cars_ahead.push_back({predictions.current_sf[i][0], distance}); // veh_id, distance
      }
    } else if (collision_adjuscent) { //vehicles in adjacent lanes attempting to change lane in a short distance
      double distance = predictions.current_sf[i][5] - (car_s+behind);
      if (distance < 0) distance = 6945.554 + distance;
      if (distance < buffer) { //veh ahead
        cars_ahead.push_back({predictions.current_sf[i][0], distance}); // veh_id, distance
      }
    }
  }
  // sorting vehicles by distance ahead
  std::sort(cars_ahead.begin(), cars_ahead.end(),
            [](const std::vector<double>& a, const std::vector<double>& b) {
                return a[1] < b[1];
            });
  return cars_ahead;
}

vector<string> BehavPlan::GetNextStates(string cur_state){
  return next_states[cur_state];
}

vector<vector<double>> BehavPlan::GetBestTrajectory(double car_s,
                                                    double car_d,
                                                    vector<vector<double>> sensor_fusion,
                                                    vector<double> previous_path_x,
                                                    vector<double> previous_path_y){

  static vector<vector<double>>path_backup = {{},{},{},{},{}};//saving s,v,d,d_time,shift in case of emergency breaking
  static double v_max = 22.12848; // speed limit for current time step
  int path_length = 100; // number of planned path points
  int path_emerg = 10; // number of planned path points in case of emergency breaking

  static bool shift = false; // changin lane flag
  static tk::spline d_t; // spline for lane shift

  vector<double> next_x_vals; // next path x points
  vector<double> next_y_vals; // next path y points

  int prev_size; // length of the previous path
  double buffer = 20; // distance to the car ahead
  static double s0 = car_s; //s of the last planned path point
  double s1 = s0;
  static double v0 = 0; //v of last planned path point
  double v = v0;
  double a; // a_s
  const double t = 0.02; // time step for path points
  static double d_time = 0.02; // for d spline during shift
  static double d_next = car_d; // d of the last planned path point
  static double d_shift = d_next; // d for target lane during shift
  int current_lane = (int)(d_next / 4); // lane of the last planned path point
  static int target_lane = current_lane; // target lane during shift
  double veh_id = -1; // id of vehicle ahead, -1 if no veh ahead

  // cars_ahead - car's ahead in lane incl. unexpected shift from adjacent lanes
  auto cars_ahead = CarsAhead(car_s, current_lane, 0, 40, true);

  prev_size = previous_path_x.size();
  if (cars_ahead.size()>0) {
    double distance;
    double v_veh0 = predictions.vehicles[cars_ahead[0][0]].vs;
    double a_veh0 = predictions.vehicles[cars_ahead[0][0]].as;
    distance = cars_ahead[0][1];
    v_max = min(v_veh0 + a_veh0, 22.12848);
    veh_id = cars_ahead[0][0];

    if (distance < buffer ) { //emergency breaking
      int diff = path_length - prev_size; // path_points used

      prev_size = min(path_emerg, prev_size);
      if (prev_size == path_emerg ){

        s0 = path_backup[0][path_emerg-1+diff];
        v0 = path_backup[1][path_emerg-1+diff];
        d_next = path_backup[2][path_emerg-1+diff];
        d_time = path_backup[3][path_emerg-1+diff];
        shift = path_backup[4][path_emerg-1+diff];
        for (auto j=0;j<5;j++){
          path_backup[j].erase(path_backup[j].begin()+path_emerg+diff,path_backup[j].end());
        }
      }
    }
    // consider shift
    if(!shift){
      vector<int> next_lanes;
      if (current_lane == 0) next_lanes={1};
      else if (current_lane == 1) next_lanes={0,2};
      else if (current_lane == 2) next_lanes={1};

      for (int i = 0; i < next_lanes.size(); i++){
        auto cars_ahead_lane = CarsAhead(car_s, next_lanes[i], -10, 60);
        if (cars_ahead_lane.size() == 0) {
          shift = true;
          target_lane = next_lanes[i];
          d_shift = 4 * target_lane + 2;
          if (d_shift == 10) d_shift = 9.8;

          vector<double> spline_t_points, spline_d_points;
          spline_t_points = {-1, 0, 0.02, 4.02, 4.04, 5, 6};
          spline_d_points = {d_next, d_next, d_next, d_shift, d_shift, d_shift, d_shift};

          d_t.set_points(spline_t_points, spline_d_points);
          break;
        }
      }
    }
  } else v_max = 22.12848;

  // trajectory planning

  if (prev_size > 0){ // filling path with old points
    for (auto i = 0; i < prev_size; i++){
      next_x_vals.push_back(previous_path_x[i]);
      next_y_vals.push_back(previous_path_y[i]);
    }
  }

  for (auto i = prev_size; i < path_length; i++) { // filling path with new points
    double vd = 0;//substitude of a_t and a_n to decrease a_s during lane shift
    if (shift == true) { // shift in process
      d_time += 0.02;
      d_next = d_t(d_time);
      vd = 1;
      if (d_time > 4.02){ // shift is finished
        shift = false;
        d_time = 0;
        vd = 0;
      }
    } else {
      d_next = (4 * current_lane + 2);
      if (current_lane == 2) d_next = 9.8;
    }
    a = 8;
    double a_vmax = a;
    double a_buf = a;
    // estimating next path planning using optimistic a and v
    s1 = s0 + v0 * t + a * t * t * 0.5;
    v = v0 + a * t;

    // v_s correction considering curve
    // calculating v_x, v_y as derivatives dx/dt = dx/ds * ds/dt
    double x_dot_s, y_dot_s; // calculating v_s to adjust v on curves
    x_dot_s = track.x_s.deriv(1,s1) + (d_next) * track.dx_s.deriv(1,s1);
    y_dot_s = track.y_s.deriv(1,s1) + (d_next) * track.dy_s.deriv(1,s1);
    double v_coef = sqrt(x_dot_s*x_dot_s + y_dot_s*y_dot_s);

    // checking max speed considering v_s correction
    if (v * v_coef > sqrt(v_max*v_max-vd*vd)) {
      v = v_max / v_coef;
      a_vmax = (v - v0) / t;
      if (a_vmax < -8) { // min acceleration
        a_vmax = -8;
      }
    }
    // calculating new path point considering collision safe buffer
    if (veh_id > -1){ // there's a vehicle ahead
      double veh_s = predictions.GetVehSinT(veh_id, i * t);
      double distance = veh_s - s1;
      if (distance < 0) distance = 6945.554 + distance;
      if (distance < buffer){
        a_buf = (buffer - s0 - v0 * t) * 0.5 / (t * t);
        if (a_buf < -8){
          a_buf = -8;
        }
      }
    }
    // calculating next path point considering collision safe buffer ahead and max speed constraints
    a = min(a_vmax, a_buf);
    v = v0 + a * t;
    s1 = s0 + v0 * t + a * t * t * 0.5;
    a = 8;

    s0 = s1;
    v0 = v;
    s0 = fmod(s0, 6945.554);
    // storing s,v,d,d_time,shift in case of emergency breaking
    path_backup[0].push_back(s0);
    path_backup[1].push_back(v0);
    path_backup[2].push_back(d_next);
    path_backup[3].push_back(d_time);
    path_backup[4].push_back((double)shift);
    if (path_backup[0].size()>path_length){
      for (auto j=0;j<5;j++) path_backup[j].erase(path_backup[j].begin());
    }

    // push new path points to the result
    vector<double> nextXY;
    nextXY = track.getXY(s0, d_next);
    next_x_vals.push_back(nextXY[0]);
    next_y_vals.push_back(nextXY[1]);
  }
  return {next_x_vals, next_y_vals};
}

/*
vector<vector<double>> BehavPlan::Next(){
  static string cur_state = "KL";
  vector <string> next_states = BehavPlan::GetNextStates(cur_state);

  vector<double> cost_for_states(next_states.size());
  vector<vector<vector<double>>> trajectory_for_state(next_states.size());

  for (auto i=0; i<next_states.size(); i++){
    trajectory_for_state[i] = GetTrajectory(next_states[i]);
    cost_for_states[i] = BehavPlan::GetCost(trajectory_for_state[i]);
  }

  int ind_min_cost = GetIndexMinCost(cost_for_states);
  return trajectory_for_state[ind_min_cost];
}
*/

//
// Helper
//
int GetIndexMinCost(vector<double> cost_for_states){
  int ind_min_cost = 0;
  double min_cost = std::numeric_limits<double>::max();
  for (auto i=0; i<cost_for_states.size(); i++){
    if (min_cost > cost_for_states[i]) {
      min_cost = cost_for_states[i];
      ind_min_cost = i;
    }
  }
  return ind_min_cost;
}


