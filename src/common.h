#ifndef COMMON_H
#define COMMON_H
const int N_SAMPLES = 10;
const double SIGMA_SD[6] = { 10.0, 4.0, 2.0, 1.0, 1.0, 1.0 };
const double LANE_WIDTH = 4.0;
const int LANE_COUNT = 3;
const int frames_per_sec = 50;				// 1 step / 0.02 sec.	

#define USE_LOGGING
//#define GT 

#ifdef GT
	const double SPEED_LIMIT = 100.0 / 2.237;					// 50 MPH in m/s
	const double FOLLOWING_SPEED_DIFF = 0;
	const double SPEED_ALIGN_MULTIPLER = 4.0;
	const double MIN_DIST_FROM_CAR_IN_FRONT_OF_US = 10.0;
	const double SAFE_DISTANCE_FOR_LANE_CHANGE_AHEAD = 20;
	const double SAFE_DISTANCE_FOR_LANE_CHANGE_BEHIND = 5.5;
	const double CONVENIENCE_COST = 0.001;
	const double MIN_DURATION_BETWEEN_TWO_LANE_CHANGES = 3;

#else 
	const double SPEED_LIMIT = 50.0 / 2.237;					// 50 MPH in m/s
	const double FOLLOWING_SPEED_DIFF = 0.5;
	const double SPEED_ALIGN_MULTIPLER = 6.0;
	const double MIN_DIST_FROM_CAR_IN_FRONT_OF_US = 30.0;
	const double SAFE_DISTANCE_FOR_LANE_CHANGE_AHEAD = 30.0;
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
    VehicleState perturb() const;
};

enum StateType {
	KL,
	LCL,
	LCR
};

class HighwayState {
public:
	int lane;
	int prev_lane;
	double ref_vel;
	long int frame_cnt = 0;
	long int last_lane_changed_frame_cnt = -1;
	ofstream logfile;

	// temporary variables
	// contains valid variables only for the current advance() call.
	double car_end_s;
	double car_end_d;
	double car_end_dt;
	vector<double> prev_path_x;
	vector<double> prev_path_y;
	// current state of the car
	double car_cur_s;
	double car_cur_d;
	double car_cur_yaw;
	double car_cur_speed;
	// ------

	HighwayState();
	~HighwayState();

	void advance(const map<int, VehicleState>& predictions);
	int get_closest_ahead(double s, double d, double dt, const map<int, VehicleState>& predictions, double& dist);
	int get_closest_behind(double s, double d, double dt, const map<int, VehicleState>& predictions, double& dist);

private:
	StateType state;						// Our state at the end of the previously planned path
	const char* str(StateType st);
	const char* str_frame();
	const char* state_name(StateType st);
	void set_optimal_speed(double v);
	double calculate_cost(StateType next_state, const map<int, VehicleState>& predications, std::string* logstr);
	vector<StateType> successor_states();

	// When something unexpected happens and an other is very close or in the previously planned path:
	// - Drop previously planned path
	// - Keep in the lane where the car currently is, for at least a few seconds.
	// - don't modify the speed
	void emergency(const VehicleState& veh);

};

#endif  // COMMON_H

 