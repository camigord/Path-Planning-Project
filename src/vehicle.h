#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>

using namespace std;

class Vehicle {
public:

  map<string, int> lane_direction = {{"PLCL", -1}, {"LCL", -1}, {"LCR", 1}, {"PLCR", 1}};

  // Check value of this
  int preferred_buffer = 6; // impacts "keep lane" behavior.

  int lane;
  double s;
  double d;
  double v;
  double a;
  string state;

  int prev_size;
  vector<vector<double>> sensor_fusion;

  double target_speed;
  int lanes_available;
  // TODO: Need a value here
  // const double max_acceleration = 0.224;

  /**
  * Constructor
  */
  Vehicle();
  Vehicle(int lane, double s, double d, double v, double a, double target_speed);

  /**
  * Destructor
  */
  virtual ~Vehicle();

  vector<string> successor_states();

  void update(double s, int prev_size, vector<vector<double>> sensor_fusion);

  vector<double> choose_next_state();

  vector<double> generate_trajectory(string state);

  vector<double> keep_lane_trajectory();

  bool get_vehicle_behind(int lane, Vehicle & rVehicle);

  bool get_vehicle_ahead(int lane, Vehicle & rVehicle);

  double get_kinematics(int lane);

  vector<double> lane_change_trajectory(string state);

  vector<double> prep_lane_change_trajectory(string state);

  double calculate_cost(string state, vector<double> parameters);

};

#endif
