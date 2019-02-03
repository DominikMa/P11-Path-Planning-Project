//
// Created by dominik on 01.02.19.
//

#include <vector>
#include "vehicle.h"

class Lane {

 public:
  int number;
  double middle_d;
  double speed;

  vector<Vehicle> cars;

  Lane(int number);
  void calc_speed();
  double speed_cars_ahead(double s);
  Vehicle getNextCar(double s);
  Vehicle getPreviousCar(double s);

};
