#include <algorithm>
#include <iostream>
#include "vehicle.h"
#include <cmath>
#include <map>
#include <string>
#include <iterator>

/**
 * Initializes Vehicle
 */

Vehicle::Vehicle(){}

Vehicle::Vehicle(int lane, double s, double d, double v, double a, double target_speed) {
    this->lane = lane;
    this->s = s;
    this->d = d;
    this->v = v;
    this->a = a;
    this->state = "KL";
    this->target_speed = target_speed;
    lanes_available = 3;
}

Vehicle::~Vehicle() {}

/*
Update information given new simulator message
*/
void Vehicle::update(double s, int prev_size, vector<vector<double>> sensor_fusion){
    this->s = s;
    this->prev_size = prev_size;
    this->sensor_fusion = sensor_fusion;
}


/*
Chooses the best next_state according to state machine and cost function
*/
vector<double> Vehicle::choose_next_state() {
    // Get possible states following state machine description
    vector<string> possible_successor_states = successor_states();

    // Keep track of the total cost of each state

    float cost;
    vector<float> costs;
    vector<vector<double>> final_results;


    for(int i = 0; i < possible_successor_states.size(); i++) {

      // Generate the trajectory we would follow if we choose this state (lane and desired speed)
      vector<double> parameters = generate_trajectory(possible_successor_states[i]);

      // Calculate the cost of that trajectory
      cost = calculate_cost(possible_successor_states[i], parameters);
      costs.push_back(cost);
      final_results.push_back(parameters);
    }

    // Find the minimum cost state
    vector<float>::iterator best_cost = min_element(begin(costs), end(costs));
    int best_idx = distance(begin(costs), best_cost);

    /*
    // For debugging
    if(state.compare(possible_successor_states[best_idx]) != 0){
      cout << state << endl;
      cout << "--------------------------------------------" << endl;
    }
    */

    // Change vehicle parameters
    this->state = possible_successor_states[best_idx];
    this->lane = final_results[best_idx][0];

    return final_results[best_idx];
}

/*
Cost is a function of the speed of the vehicle. The speed of the current lane and the speed of the "intended lane"
*/
double Vehicle::calculate_cost(string evaluated_state, vector<double> parameters){
  // Cost depends on the intended lane velocity
  int intended_lane = parameters[0] + lane_direction[evaluated_state];    // Intended lane is different for PLCL, PLCR, LCL, LCR
  double intended_speed = parameters[1];
  double cost;
  double final_speed;

  Vehicle vehicle_ahead;
  if (get_vehicle_ahead(intended_lane, vehicle_ahead)) {
      if(vehicle_ahead.s - this->s < 30.0){
        final_speed = vehicle_ahead.v;
      }
      else{
        final_speed = this->target_speed - 1/(vehicle_ahead.s - this->s);         // If the car is far away, we penalize less
      }
  }
  else{
    final_speed = this->target_speed;
  }

  cost = (2.0*this->target_speed - intended_speed - final_speed)/ this->target_speed;

  return cost;
}


vector<string> Vehicle::successor_states() {
    /*
    Provides the possible next states given the current state for the state machine
    */
    vector<string> states;

    states.push_back("KL");
    if(state.compare("KL") == 0) {
      if (lane != 0) {
        states.push_back("PLCL");
      }
      if (lane != lanes_available-1) {
        states.push_back("PLCR");
      }
    } else if (state.compare("PLCL") == 0) {
            states.push_back("PLCL");
            states.push_back("LCL");
    } else if (state.compare("PLCR") == 0) {
            states.push_back("PLCR");
            states.push_back("LCR");
    }
    //If state is "LCL" or "LCR", then just return "KL"
    return states;
}

vector<double> Vehicle::generate_trajectory(string state) {
    /*
    Given a possible next state, generate the appropriate trajectory to realize the next state.
    */
    vector<double> parameters;
    if (state.compare("KL") == 0) {
        parameters = keep_lane_trajectory();
    } else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
        parameters = lane_change_trajectory(state);
    } else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
        parameters = prep_lane_change_trajectory(state);
    }
    return parameters;
}

vector<double> Vehicle::keep_lane_trajectory() {
    /*
    Generate a keep lane trajectory.
    */
    double new_velocity = get_kinematics(this->lane);

    return {double(this->lane), new_velocity};
}

double Vehicle::get_kinematics(int lane) {
    /*
    Gets next timestep kinematics (position, velocity, acceleration)
    for a given lane. Tries to choose the maximum velocity and acceleration,
    given other vehicle positions and accel/velocity constraints.
    */
    double new_velocity;

    Vehicle vehicle_ahead;

    if (get_vehicle_ahead(lane, vehicle_ahead)) {
        if(vehicle_ahead.s - this->s < 30.0){
            new_velocity = min(vehicle_ahead.v, this->target_speed);
        }
        else{
            new_velocity = this->target_speed;
        }

    } else {
        new_velocity = this->target_speed;
    }

    return new_velocity;
}

bool Vehicle::get_vehicle_behind(int current_lane, Vehicle & rVehicle) {
    /*
    Returns a true if a vehicle is found behind the current vehicle, false otherwise. The passed reference
    rVehicle is updated if a vehicle is found.
    */

    int max_s = -1;
    bool found_vehicle = false;
    for(int i=0; i<sensor_fusion.size(); i++){
      // Check if car is in our lane
      float d = sensor_fusion[i][6];
      if(d < (2+4*current_lane+2) && d > (2+4*current_lane-2)){
          double vx = sensor_fusion[i][3];
          double vy = sensor_fusion[i][4];
          double check_speed = sqrt(vx*vx + vy*vy);  // Velocity Magnitude
          double check_car_s = sensor_fusion[i][5];

          check_car_s += ((double)prev_size * 0.02 * check_speed);    // Predict current position of car based on the speed it was traveling at

          // Check s values are greater than mine and s gap (is car in front of me?)
          if(check_car_s < this->s && check_car_s > max_s){
              max_s = check_car_s;
              Vehicle temp_vehicle = Vehicle(current_lane, check_car_s, 0.0, check_speed, 0.0, 0.0);
              rVehicle = temp_vehicle;
              found_vehicle = true;
          }
      }
    }

    return found_vehicle;
}

bool Vehicle::get_vehicle_ahead(int current_lane, Vehicle & rVehicle) {
    /*
    Returns a true if a vehicle is found ahead of the current vehicle, false otherwise. The passed reference
    rVehicle is updated if a vehicle is found.
    */

    int min_s = 6945;
    bool found_vehicle = false;
    for(int i=0; i<sensor_fusion.size(); i++){
      // Check if car is in our lane
      float d = sensor_fusion[i][6];
      if(d < (2+4*current_lane+2) && d > (2+4*current_lane-2)){
          double vx = sensor_fusion[i][3];
          double vy = sensor_fusion[i][4];
          double check_speed = sqrt(vx*vx + vy*vy);  // Velocity Magnitude
          double check_car_s = sensor_fusion[i][5];

          check_car_s += ((double)prev_size * 0.02 * check_speed);    // Predict current position of car based on the speed it was traveling at

          // Check s values are greater than mine and s gap (is car in front of me?)
          if(check_car_s > this->s && check_car_s < min_s){
              min_s = check_car_s;
              Vehicle temp_vehicle = Vehicle(current_lane, check_car_s, 0.0, check_speed, 0.0, 0.0);
              rVehicle = temp_vehicle;
              found_vehicle = true;
          }
      }
    }

    return found_vehicle;
}

vector<double> Vehicle::prep_lane_change_trajectory(string state) {
    /*
    Generate a trajectory preparing for a lane change.
    */
    double new_velocity;

    Vehicle vehicle_behind;
    int new_lane = this->lane + lane_direction[state];

    double current_lane_vel = get_kinematics(this->lane);

    if (get_vehicle_behind(this->lane, vehicle_behind)) {
        //Keep speed of current lane so as not to collide with car behind.
        new_velocity = current_lane_vel;

    } else {
        double next_lane_velocity = get_kinematics(new_lane);
        new_velocity = min(next_lane_velocity, current_lane_vel);
    }

    return {double(this->lane), new_velocity};
}

vector<double> Vehicle::lane_change_trajectory(string state) {
    /*
    Generate a lane change trajectory.
    */
    int new_lane = this->lane + lane_direction[state];
    double new_velocity;

    for(int i=0; i<sensor_fusion.size(); i++){
      // Check if car is in our lane
      float d = sensor_fusion[i][6];
      if(d < (2+4*new_lane+2) && d > (2+4*new_lane-2)){
          double vx = sensor_fusion[i][3];
          double vy = sensor_fusion[i][4];
          double check_speed = sqrt(vx*vx + vy*vy);  // Velocity Magnitude
          double check_car_s = sensor_fusion[i][5];

          check_car_s += ((double)prev_size * 0.02 * check_speed);    // Predict current position of car based on the speed it was traveling at

          // Can we change lane?
          if((check_car_s > this->s - 5) && (check_car_s < this->s + 30)){
              new_velocity = get_kinematics(this->lane);
              return {double(this->lane), new_velocity};
          }
      }
    }

    new_velocity = get_kinematics(new_lane);

    return {double(new_lane), new_velocity};
}
