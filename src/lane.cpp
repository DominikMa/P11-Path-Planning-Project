//
// Created by dominik on 01.02.19.
//

#include "lane.h"

const double max_lookout_dist = 50.0;

Lane::Lane(int number) {
  this->number = number;
  middle_d = 2 + 4 * number;
}
void Lane::calc_speed() {
  double tmp_speed = 0.0;
  double cars_count = 0.0;
  for (const auto &car : cars) {
    tmp_speed += car.v;
    cars_count += 1.0;
  }
  if (cars_count == 0) speed = 100;
  else speed = tmp_speed / cars_count;
}
double Lane::speed_cars_ahead(double s) {
  double speed = 0.0;
  double cars_count = 0.0;
  for (auto &car : cars) {
    if (car.s >= s - 0.1 && car.s - s < max_lookout_dist) {
      speed += car.v;
      cars_count += 1.0;
    }
  }
  if (cars_count == 0) return 100.0;
  return speed / cars_count;
}

Vehicle Lane::getNextCar(double s) {
  double dst = 1000.0;
  Vehicle nextCar = Vehicle(-1, 0, 0, 0);
  for (auto &car : cars) {
    double tmp_dst = car.s - s + 0.1;
    if (tmp_dst > 0.0 && tmp_dst < dst) {
      nextCar = car;
      dst = tmp_dst;
    }
  }
  return nextCar;
}
Vehicle Lane::getPreviousCar(double s) {
  double dst = 1000.0;
  Vehicle previousCar = Vehicle(-1, 0, 0, 0);
  for (auto &car : cars) {
    double tmp_dst = s - car.s + 0.1;
    if (tmp_dst >= 0.0 && tmp_dst < dst) {
      previousCar = car;
      dst = tmp_dst;
    }
  }
  return previousCar;
}
