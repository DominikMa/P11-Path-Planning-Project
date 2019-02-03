//
// Created by dominik on 31.01.19.
//

#include <cmath>
#include "vehicle.h"
Vehicle::Vehicle(double id, double vx, double vy, double s, double d) {
  this->id = (int) (id);
  this->s = s;
  this->d = d;
  this->v = calcSpeed(vx, vy);
  calcLanes();

}

Vehicle::Vehicle(double id, double v, double s, double d) {
  this->id = (int) (id);
  this->v = v;
  this->s = s;
  this->d = d;
  calcLanes();
}

double Vehicle::calcSpeed(double vx, double vy) {
  return sqrt(vx * vx + vy * vy);
}

void Vehicle::calcLanes() {
  double e = 0.6;
  if (d >= 0.0 - e && d < 4.0 + e) lanes.push_back(0);
  if (d >= 4.0 - e && d < 8.0 + e) lanes.push_back(1);
  if (d >= 8.0 - e && d < 12.0 + e) lanes.push_back(2);
}

void Vehicle::predict(double sec) {
  s = s + sec * v;
}

void Vehicle::predict(int nrWaypoints) {
  double sec = (double) (nrWaypoints) * 0.02;
  predict(sec);
}

int Vehicle::closestLaneNumber() {
  int closes_lane_number = lanes[0];
  // If in more than one lane keep closest lane
  if (lanes.size() > 1) {
    closes_lane_number = floor(d/4.0);
  }
  return closes_lane_number;
}
