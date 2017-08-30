//
// Created by Oleg Leyzerov on 19/08/2017.
//

#ifndef PATH_PLANNING_PREDICTIONS_H
#define PATH_PLANNING_PREDICTIONS_H

#include "Track.h"

#include <vector>
#include <chrono>
#include <map>
using namespace std;

class Vehicle {

public:
    vector <vector <double>> vehStates; // 0){time}, 1){x}, 2){y}, 3){vx}, 4){vy}, 5){s}, 6){d}
    double as, ad;
    double vs, vd;

    Vehicle() : vehStates(6,vector<double>(0.0)) {}
    ~Vehicle() {}
    vector<vector<double>> getState();
};

class Predictions {


public:
  map <int, Vehicle> vehicles;
  vector<vector<double>> current_sf; // current sensor fusion vector
  Predictions()  {}
  ~Predictions() {}
  void update (Track track, vector<vector<double>>sensor_fusion);
  double GetEpsi (Track track, double vx, double vy, double s, double d);
  vector<int> getVehList();
  double GetVehSinT(int vehID, double t);
  vector<int> CarsInLane(double car_s, double car_d);
};


#endif //PATH_PLANNING_PREDICTIONS_H
