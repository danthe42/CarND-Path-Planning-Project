#ifndef COMMON_H
#define COMMON_H
const int N_SAMPLES = 10;
const double SIGMA_SD[6] = { 10.0, 4.0, 2.0, 1.0, 1.0, 1.0 };
const double LANE_WIDTH = 4.0;
const int LANE_COUNT = 3;
const int frames_per_sec = 50;				// 1 step / 0.02 sec.	

//#define GT 

#ifdef GT
	const double SPEED_LIMIT = 100.0 / 2.237;					// 50 MPH in m/s
	const double FOLLOWING_SPEED_DIFF = 0;
	const double SPEED_ALIGN_MULTIPLER = 4.0;
	const double MIN_DIST_FROM_CAR_IN_FRONT_OF_US = 10.0;
	const double SAFE_DISTANCE_FOR_LANE_CHANGE = 7.5;
	const double CONVENIENCE_COST = 0.001;
	const double MIN_DURATION_IN_A_LANE = 3;

#else 
	const double SPEED_LIMIT = 50.0 / 2.237;					// 50 MPH in m/s
	const double FOLLOWING_SPEED_DIFF = 0.05;
	const double SPEED_ALIGN_MULTIPLER = 6.0;
	const double MIN_DIST_FROM_CAR_IN_FRONT_OF_US = 30.0;
	const double SAFE_DISTANCE_FOR_LANE_CHANGE_AHEAD = 25;
	const double SAFE_DISTANCE_FOR_LANE_CHANGE_BEHIND = 7.5;
	const double CONVENIENCE_COST = 0.01;
	const double MIN_DURATION_BETWEEN_TWO_LANE_CHANGES = 6;
#endif 

const double OPTIMAL_SPEED = SPEED_LIMIT * 0.95;					

#include <map>
#include <vector>
#include "helpers.h"

using std::vector;
using std::map;

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

struct Trajectory {
	vector<double> vals;
	Trajectory(vector<double> s_coeffs, vector<double> d_coeffs, double t) {
		vals = { s_coeffs[0], s_coeffs[1], s_coeffs[2], d_coeffs[0], d_coeffs[1], d_coeffs[2], t };
	}
	Trajectory() {
		vals = { 0,0,0,0,0,0,0 };
	}
    Trajectory(vector<double> start_s, vector<double> start_d, VehicleState& goal);

	static Trajectory Generate(vector<double> start_s, vector<double> start_d, int target_vehicle, vector<double> delta, double T, const map<int, VehicleState>& predictions);

private:
	static vector<double> JMT(vector<double>& start, vector<double>& end, double T);

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

	// temporary variables
	// contains valid variables only for the current advance() call.
	double car_end_s;
	double car_end_d;
	double car_speed;
	double car_end_dt;
	vector<double> prev_path_x;
	vector<double> prev_path_y;
	double car_cur_s;
	double car_cur_d;
	// ------

	HighwayState();
	Trajectory advance(double _car_end_s, double _car_end_d, double _car_speed, double _car_end_dt, const map<int, VehicleState>& predictions);
	int get_closest_ahead(double s, double d, double dt, const map<int, VehicleState>& predictions, double& dist);
	int get_closest_behind(double s, double d, double dt, const map<int, VehicleState>& predictions, double& dist);

private:
	StateType state;						// Our current state

	void set_optimal_speed(double v);
	double calculate_cost(StateType next_state, const map<int, VehicleState>& predications);
	vector<StateType> successor_states();
};

double calculate_cost(const Trajectory& tr, int target_vehicle, vector<double> delta, double T, const map<int, VehicleState>& predications);

#endif  // COMMON_H

 