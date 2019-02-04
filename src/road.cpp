//
// Created by dominik on 01.02.19.
//

#include <iostream>
#include <cmath>
#include <limits>
#include "road.h"

// Set the maximum speed of the road
const double road_max_speed = 49.9;
const double road_max_speed_mps = road_max_speed / 2.212;

// Set how aggressively we change lanes
const double change_next_fact = 0.1;
const double change_previous_fact = 0.2;
// Set the minimum distance to change into lane
const double min_change_next_dist = 15.0;
const double min_change_previous_dist = 10.0;
// Set mimimum gained speed to change lanes
const double change_min_speed_diff = 0.3;

// Set number and distance of rough waypoints
const double waypoint_step_size = 33.0;
const int waypoint_count = 3;

// Set costs
const double cost_fact_speed = 1.0;
const double cost_fact_dist_to_best_lane = 6.0;
const double cost_fact_change = 0.1;

// Set distance to keep to next car
const double dist_to_next_car = 15.0;

Road::Road(vector<Vehicle> cars) {
  for (int i = 0; i < laneNumber; i++) {
    lanes.emplace_back(i);
  }
  for (const auto &car : cars) {
    for (const auto &lane : car.lanes) {
      lanes[lane].cars.push_back(car);
    }
  }
  for (auto &lane : lanes) {
    lane.calc_speed();
  }

}
void Road::predictAllCars(int nrWaypoints) {
  for (auto &lane : lanes) {
    for (auto &car : lane.cars) {
      car.predict(nrWaypoints);
    }
  }
}

vector<Vehicle> Road::getWaypoints(Vehicle ownCar) {
  vector<vector<Vehicle>> all_options = getOptions(ownCar);

  vector<Vehicle> best_option = all_options[0];

  int fastes_lane_number = ownCar.closestLaneNumber();
  Lane fastes_lane = lanes.at(fastes_lane_number);
  double top_speed = fastes_lane.speed_cars_ahead(ownCar.s);

  for (auto &option : all_options) {
    Vehicle first = option.at(0);
    Lane tmp_lane = lanes.at(first.closestLaneNumber());
    double tmp_lane_speed = tmp_lane.speed_cars_ahead(ownCar.s);
    if (tmp_lane_speed > top_speed + change_min_speed_diff) {
      fastes_lane = tmp_lane;
      top_speed = tmp_lane_speed;
    }
  }

  double best_cost = std::numeric_limits<double>::lowest();
  double max_speed_curren_lane = road_max_speed_mps;

  for (auto &option: all_options) {
    Vehicle first = option.at(0);
    if (fabs(first.d - ownCar.d) > 6) continue;
    for (auto &lane_number : ownCar.lanes){
      if (first.closestLaneNumber() == lane_number)
        max_speed_curren_lane = min(max_speed_curren_lane, first.v);
    }

    Lane option_lane = lanes.at(first.closestLaneNumber());
    double option_speed = option_lane.speed_cars_ahead(ownCar.s);
    double dist_to_fastest = fabs(fastes_lane.number - option_lane.number);

    double cost = cost_fact_speed * option_speed
        - cost_fact_dist_to_best_lane * dist_to_fastest
        - cost_fact_change * fabs(first.d - ownCar.d);

    if (cost > best_cost) {
      best_option = option;
      best_cost = cost;
    }
  }

  best_option.at(0).v = min(max_speed_curren_lane, best_option.at(0).v);
  return best_option;
}

vector<vector<Vehicle>> Road::getOptions(Vehicle ownCar) {

  vector<vector<Vehicle>> options;

  vector<Vehicle> keep = keepLane(ownCar);
  if (!keep.empty()) options.push_back(keep);

  for (auto &lane : lanes) {
    if (lane.number != ownCar.closestLaneNumber()) {
      vector<Vehicle> change = changeLane(ownCar, lane);
      if (!change.empty()) options.push_back(change);
    }
  }

  return options;
}
vector<Vehicle> Road::keepLane(Vehicle ownCar) {
  vector<Vehicle> waypoints;

  // If in more than one lane keep closest lane
  int closes_lane_number = ownCar.closestLaneNumber();

  Lane own_lane = lanes.at(closes_lane_number);
  // Get maximum speed to drive with
  double max_speed = getMaxSpeed(own_lane, ownCar.s);

  // Generate cars in 30 m steps, with adjusted speed as vector and return
  double step_size = 30.0;
  int waypoint_count = 3;

  for (int i = 1; i <= waypoint_count; i++) {
    Vehicle next_waypoint = Vehicle(ownCar.id, max_speed,
                                    ownCar.s + i * step_size, own_lane.middle_d);
    waypoints.push_back(next_waypoint);
  }

  return waypoints;
}

double Road::getMaxSpeed(Lane lane, double s) {
  vector<Vehicle> nextCars;
  Vehicle tmpNextCar = lane.getNextCar(s);
  if (tmpNextCar.id != -1) {
    nextCars.push_back(tmpNextCar);
  }

  double max_speed = road_max_speed_mps;
  for (auto &car : nextCars) {
    if (car.s - s < 60) {
      double diff = (car.s - s - dist_to_next_car);
      double goal_speed = car.v;
      if (diff <= 0) {
        double c;
        if (diff >= -(dist_to_next_car/15.0)){
          c = 0.1*diff;
        } else if (diff >= -(dist_to_next_car/6.0)){
          c = 0.3*diff;
        } else if (diff >= -(dist_to_next_car/3.0)){
          c = 0.4*diff;
        } else if (diff >= -(dist_to_next_car/1.2)){
          c = 0.8*diff;
        } else {
          c = diff;
        }
        goal_speed -= c;
      } else {
        double c;
        if (diff <= dist_to_next_car/7){
          c = 0.05*diff;
        } else if (diff <= dist_to_next_car/2){
          c = 0.2*diff;
        } else {
          c = 0.4*diff;
        }
        goal_speed += c;
      }
      max_speed = min(max_speed, goal_speed);
    }
  }
  return max_speed;
}
vector<Vehicle> Road::changeLane(Vehicle ownCar, Lane lane) {
  vector<Vehicle> waypoints;

  // check for free in front
  Vehicle next_car = lane.getNextCar(ownCar.s);
  if (next_car.id != -1) {
    double speed_diff = ownCar.v - next_car.v;
    double dist_diff = max(fabs(next_car.s - ownCar.s), 0.01);
    if (dist_diff < min_change_next_dist) return waypoints;
    if (speed_diff / dist_diff > change_next_fact) return waypoints;
  }

  // check for free space in back
  Vehicle previous_car = lane.getPreviousCar(ownCar.s);
  if (previous_car.id != -1) {
    double speed_diff = previous_car.v - ownCar.v;
    double dist_diff = max(fabs(previous_car.s - ownCar.s), 0.01);
    if (dist_diff < min_change_previous_dist) return waypoints;
    if (speed_diff / dist_diff > change_previous_fact) return waypoints;
  }

  // get max speed for lane
  double max_speed = getMaxSpeed(lane, ownCar.s);

  // generate waypoints
  for (int i = 1; i <= waypoint_count; i++) {
    Vehicle next_waypoint = Vehicle(ownCar.id, max_speed,
                                    ownCar.s + i * waypoint_step_size, lane.middle_d);
    waypoints.push_back(next_waypoint);
  }

  return waypoints;
}

