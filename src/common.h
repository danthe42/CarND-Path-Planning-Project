#ifndef COMMON_H
#define COMMON_H
const double LANE_WIDTH = 4.0;
const int LANE_COUNT = 3;
const int frames_per_sec = 50;						// Number of frames per second: 1/0.02 sec.	

#define USE_LOGGING								// development logging: both to log.txt and to console.
//#define GT										// Race car mode: 100 MPH speed limit, smaller safety limits, dangerous maneuvers, ... Not safe, just for fun :) 

#ifdef GT
	const double SPEED_LIMIT = 100.0 / 2.237;					// 100 MPH in m/s
	const double FOLLOWING_SPEED_DIFF = 0;
	const double SPEED_ALIGN_MULTIPLER = 4.0;
	const double MIN_DIST_FROM_CAR_IN_FRONT_OF_US = 10.0;
	const double SAFE_DISTANCE_FOR_LANE_CHANGE_AHEAD = 3;
	const double SAFE_DISTANCE_FOR_LANE_CHANGE_BEHIND = 3;
	const double CONVENIENCE_COST = 0.001;
	const double MIN_DURATION_BETWEEN_TWO_LANE_CHANGES = 1;
	const double MAX_SPEED_DIFF_AT_CHANGING_LANES = 3;

#else 
	const double SPEED_LIMIT = 50.0 / 2.237;					// 50 MPH in m/s
	const double FOLLOWING_SPEED_DIFF = 0.5;
	const double SPEED_ALIGN_MULTIPLER = 6.0;
	const double MIN_DIST_FROM_CAR_IN_FRONT_OF_US = 30.0;
	const double SAFE_DISTANCE_FOR_LANE_CHANGE_AHEAD = 5.0;
	const double SAFE_DISTANCE_FOR_LANE_CHANGE_BEHIND = 5.0;
	const double CONVENIENCE_COST = 0.01;
	const double MIN_DURATION_BETWEEN_TWO_LANE_CHANGES = 6;
	const double MAX_SPEED_DIFF_AT_CHANGING_LANES = 5;
#endif 

const double OPTIMAL_SPEED = SPEED_LIMIT * 0.99;					

#include <map>
#include <vector>
#include <iostream>
#include <fstream>
#include "helpers.h"

using std::vector;
using std::map;
using std::ofstream;

struct VehicleState {
	double s, sdot, s2dot;
	double d, ddot, d2dot;
	double t;
	VehicleState(const VehicleState& vs) : s(vs.s), sdot(vs.sdot), s2dot(vs.s2dot), d(vs.d), ddot(vs.ddot), d2dot(vs.d2dot), t(vs.t) {};
	VehicleState(double _s, double _sdot, double _s2dot, double _d, double _ddot, double _d2dot, double _t = 0)
		: s(_s), sdot(_sdot), s2dot(_s2dot)
		, d(_d), ddot(_ddot), d2dot(_d2dot)
		, t(_t)
	{}

	// predict what state the car be in, at time T
    VehicleState state_in(double T) const;
	VehicleState offset(vector<double> delta, double dt = 0.0) const;
};

enum StateType {
	KL,				// State: Keep Lane 
	LCL,			// State: Lane change to Left
	LCR				// State: Lane change to right
};

class HighwayState {
public:
	// the index of the lane we want to drive in
	int lane;				

	// the speed we want to use
	double ref_vel;

	// Global frame counter: for timer and logging purposes
	long int frame_cnt = 0;

	// We start a timer after a lane change maneuver. If this timer is active, this attribute will contain the timestamp of the maneuver. Otherwise it's -1. 
	long int last_lane_changed_frame_cnt = -1;

	// current state of the car
	double car_cur_s;
	double car_cur_d;
	double car_cur_yaw;
	double car_cur_speed;

	// temporary variables
	// contains valid variables only for the current advance() call.
	double car_end_s;
	double car_end_d;
	double car_end_dt;
	vector<double> prev_path_x;
	vector<double> prev_path_y;
	// ------

	HighwayState();
	~HighwayState();

	void advance(const map<int, VehicleState>& predictions);
	int get_closest_ahead(double s, double d, double dt, const map<int, VehicleState>& predictions, double& dist);
	int get_closest_behind(double s, double d, double dt, const map<int, VehicleState>& predictions, double& dist);

private:

	// The current state 
	StateType state;						// Our state at the end of the previously planned path

	// Set our speed target value
	void set_optimal_speed(double v);

	//
	double calculate_cost(StateType next_state, const map<int, VehicleState>& predications, std::string* logstr);

	// Get the list of possible successor states.
	vector<StateType> successor_states();

	// Safety issue happened. When something unexpected happens, or an other vehicle is very close, or is in our previously planned path:
	// - Drop previously planned path
	// - Keep in the lane where the car currently is, for at least a few seconds.
	// - set the speed target to 75% of the vehicle responsible for this emergency 
	// The whole path in front of us will be planned again from the scratch, using these new parameters.
	void emergency(const VehicleState& veh);

	// helpers for logging 
	ofstream logfile;
	const char* str(StateType st);
	const char* str_frame();
	const char* state_name(StateType st);

};

#endif  // COMMON_H

 