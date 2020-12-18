#ifndef COMMON_H
#define COMMON_H
const int N_SAMPLES = 10;
const double SIGMA_SD[6] = { 10.0, 4.0, 2.0, 1.0, 1.0, 1.0 };

#include <map>
#include <vector>
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
    VehicleState offset(vector<double> delta) const;
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

double calculate_cost(const Trajectory& tr, int target_vehicle, vector<double> delta, double T, const map<int, VehicleState>& predications);

#endif  // COMMON_H

 