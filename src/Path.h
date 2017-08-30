//
// Created by Oleg Leyzerov on 19/08/2017.
//

#ifndef PATH_PLANNING_PATH_H
#define PATH_PLANNING_PATH_H

#include <vector>
#include <map>
#include "Track.h"
#include "Predictions.h"


using namespace std;

class BehavPlan{
public:
    Track track;
    map<string,vector<string>> next_states;
    Predictions predictions;
    BehavPlan() {
      /*
      next_states["KL"] = {"KL", "PLCL", "PLCR"};
      next_states["PLCL"] = {"LCL", "PLCL", "KL"};
      next_states["PLCR"] = {"LCR", "PLCR", "KL"};
      next_states["LCL"] = {"LCL", "KL"};
      next_states["LCR"] = {"LCR", "KL"};
       */
    }
    ~BehavPlan() {}
    vector<string> GetNextStates(string cur_state);
    vector<vector<double>> GetBestTrajectory(double car_s,
                                                        double car_d,
                                                        vector<vector<double>> sensor_fusion,
                                                        vector<double> previous_path_x,
                                                        vector<double> previous_path_y);

    void Init(vector<double> map_waypoints_x,
              vector<double> map_waypoints_y,
              vector<double> map_waypoints_s,
              vector<double> map_waypoints_dx,
              vector<double> map_waypoints_dy);

    vector<vector<double>> CarsAhead(double car_s, int lane, double behind=0, double length=100,bool collision=false);
};

class Path {
public:
    BehavPlan behav_plan;

    Path(vector<double> map_waypoints_x,
         vector<double> map_waypoints_y,
         vector<double> map_waypoints_s,
         vector<double> map_waypoints_dx,
         vector<double> map_waypoints_dy);

    ~Path() {}

    vector<vector<double>> Process(double car_s,
                                   double car_d,
                                   vector<vector<double>> sensor_fusion,
                                   vector<double, allocator<double>> previous_path_x,
                                   vector<double, allocator<double>> previous_path_y);
};

#endif //PATH_PLANNING_PATH_H
