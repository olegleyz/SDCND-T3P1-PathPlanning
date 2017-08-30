//
// Created by Oleg Leyzerov on 19/08/2017.
//
#include "spline.h"
#include "Predictions.h"
#include "Track.h"
#include <iostream>
#include <cmath>

using namespace std;

double Predictions::GetEpsi (Track track, double vx, double vy, double s, double d){
  vector<double> wp1 = track.getXY(s+10, d);
  vector<double> wp0 = track.getXY(s-10, d);
  double track_heading = atan2(wp1[1]-wp0[1], wp1[0]-wp0[0]);
  double veh_heading = atan2(vy, vx);
  return (veh_heading - track_heading);
}

void Predictions::update (Track track, vector<vector<double>>sensor_fusion){
  current_sf = sensor_fusion;
  for (auto i=0; i<sensor_fusion.size();i++){
    if (sensor_fusion[i][6]>=0) { // don't consider vehicles with d < 0
      int veh_id = (int) sensor_fusion[i][0];
      if (!vehicles.count(veh_id)) { // if car_id is not tracked before
        Vehicle veh;
        vehicles[veh_id] = veh;
      }
      for (auto j = 0; j < 6; j++) {
        vehicles[veh_id].vehStates[j].push_back(sensor_fusion[i][j + 1]);//x, y, vx, vy, s, d

        if (vehicles[veh_id].vehStates[j].size() > 2) // save just 2 SF states
          vehicles[veh_id].vehStates[j].erase(vehicles[veh_id].vehStates[j].begin());
      }
      double vx0, vy0, s0, d0, v0, epsi0;
      vx0 = vehicles[veh_id].vehStates[2][0];
      vy0 = vehicles[veh_id].vehStates[3][0];
      s0 = vehicles[veh_id].vehStates[4][0];
      d0 = vehicles[veh_id].vehStates[5][0];

      v0 = sqrt(pow(vx0, 2) + pow(vy0, 2));
      epsi0 = GetEpsi(track, vx0, vy0, s0, d0);
      vehicles[veh_id].vs = v0 * cos(fabs(epsi0));
      if (vehicles[veh_id].vehStates[0].size() == 1) vehicles[veh_id].vd = 0;
      vehicles[veh_id].as = 0;
      vehicles[veh_id].ad = 0;

      if (vehicles[veh_id].vehStates[0].size() == 2) { // calculate a and v

        double vx1, vy1, s1, d1, v1, epsi1;
        vx1 = vehicles[veh_id].vehStates[2][1];
        vy1 = vehicles[veh_id].vehStates[3][1];
        s1 = vehicles[veh_id].vehStates[4][1];
        d1 = vehicles[veh_id].vehStates[5][1];

        v1 = sqrt(pow(vx1, 2) + pow(vy1, 2));
        epsi1 = GetEpsi(track, vx1, vy1, s1, d1);

        double t = (s1 - fmod(s0,6945.554)) / (0.5 * (v1 + v0));
        vehicles[veh_id].as = (v1 * cos(fabs(epsi1)) - v0 * cos(fabs(epsi0))) / t;
        vehicles[veh_id].vs = v1 * cos(fabs(epsi1));
        vehicles[veh_id].ad = (d1-d0)/t - vehicles[veh_id].vd;
        vehicles[veh_id].vd = (d1-d0)/t;
      }
    }
  }
}

double Predictions::GetVehSinT(int vehID, double t) {
  double veh_s1 = 0;

  if (vehicles.count(vehID)) {
    Vehicle veh = vehicles[vehID];
    double veh_s0 = veh.vehStates[4][1];
    double veh_vs0 = veh.vs;
    double veh_as = veh.as;

    veh_s1 = veh_s0 + veh_vs0 * t + veh_as * t *t * 0.5;
  }
  return veh_s1;
}
