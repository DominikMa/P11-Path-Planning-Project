//
// Created by dominik on 01.02.19.
//


#include <vector>
#include "lane.h"

class Road {
 public:
  static const int laneNumber = 3;
  vector<Lane> lanes;


  Road(vector<Vehicle> cars);
  void predictAllCars(int nrWaypoints);

  vector<Vehicle> getWaypoints(Vehicle ownCar);

 private:
  vector<vector<Vehicle>> getOptions(Vehicle ownCar);
  vector<Vehicle> keepLane(Vehicle ownCar);

  vector<Vehicle> changeLane(Vehicle ownCar, Lane lane);

  double getMaxSpeed(Lane lane, double s);
};

