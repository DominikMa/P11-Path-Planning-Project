//
// Created by dominik on 31.01.19.
//

#include <vector>

using namespace std;

class Vehicle {
 public:
  vector<unsigned int> lanes;

  int id;

  // v is in meter per second
  double v;

  double d;
  double s;

  /**
   * Constructor
   *
   * @param id
   * @param vx
   * @param vy
   * @param s
   */
  Vehicle(double id, double vx, double vy, double s, double d);
  Vehicle(double id, double v, double s, double d);
  double calcSpeed(double vx, double vy);
  void calcLanes();
  void predict(double sec);
  void predict(int nrWaypoints);
  int closestLaneNumber();
};
