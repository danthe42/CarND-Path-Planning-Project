#include <math.h>
#include <string>
#include <vector>
#include <map>
#include <unordered_map>
#include <random>
#include "Eigen-3.3/Eigen/Dense"
#include "common.h"

// for convenience
using std::string;
using std::vector;
using std::map;
using std::unordered_map;
using Eigen::MatrixXd;
using Eigen::VectorXd;

double calculate1(const Trajectory& tr, int target_vehicle, vector<double> delta, double T, const map<int, VehicleState>& predications)
{
	return -1;
}

double calculate2(const Trajectory& tr, int target_vehicle, vector<double> delta, double T, const map<int, VehicleState>& predications)
{
	return -3;
}

vector<std::pair< std::function<double(const Trajectory&, int, vector<double>, double, const map<int, VehicleState>&)>, double>> WEIGHTED_COST_FUNCTIONS = {
	{ calculate1, 2 },
	{ calculate2, 1 }
};

double calculate_cost(const Trajectory& tr, int target_vehicle, vector<double> delta, double T, const map<int, VehicleState>& predications)
{
	double cost = 0.0;
	for (auto& it : WEIGHTED_COST_FUNCTIONS) {
		double new_cost = it.first(tr, target_vehicle, delta, T, predications);
		cost += it.second * new_cost;
	}
	return cost;
}

vector<double> Trajectory::JMT(vector<double>& start, vector<double>& end, double T)
{
    /**
     * Calculate the Jerk Minimizing Trajectory that connects the initial state
     * to the final state in time T.
     *
     * @param start - the vehicles start location given as a length three array
     *   corresponding to initial values of [s, s_dot, s_double_dot]
     * @param end - the desired end state for vehicle. Like "start" this is a
     *   length three array.
     * @param T - The duration, in seconds, over which this maneuver should occur.
     *
     * @output an array of length 6, each value corresponding to a coefficent in
     *   the polynomial:
     *   s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
     *
     * EXAMPLE
     *   > JMT([0, 10, 0], [10, 10, 0], 1)
     *     [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
     */

    MatrixXd C = MatrixXd(3, 1);
    C << end[0] - (start[0] + start[1] * T + start[2] * T * T / 2),
        end[1] - (start[1] + start[2] * T),
        end[2] - start[2];

    MatrixXd A = MatrixXd(3, 3);
    A << T * T * T, T* T* T* T, T* T* T* T* T,
        3 * T * T, 4 * T * T * T, 5 * T * T * T * T,
        6 * T, 12 * T * T, 20 * T * T * T;
    MatrixXd D = A.inverse() * C;

    return { start[0],start[1],start[2] / 2,D.data()[0],D.data()[1],D.data()[2] };
}

VehicleState VehicleState::state_in(double T) const 
{
	assert(t == 0);                     // We don't handle this case, for now. 
	return VehicleState(
		s + (sdot * T) + (s2dot * T * T / 2.0),
		sdot + s2dot * T,
		s2dot,
		d + (ddot * T) + (d2dot * T * T / 2.0),
		ddot + d2dot * T,
		d2dot,
		T
	);
};

VehicleState VehicleState::offset(vector<double> delta, double dt) const
{
	return VehicleState(s + delta[0], sdot + delta[1], s2dot + delta[2], d + delta[3], ddot + delta[4], d2dot + delta[5], t);
}

VehicleState VehicleState::perturb() const
{
	std::random_device rd;
	std::mt19937 e2(rd());
	std::normal_distribution<> dist0(s, SIGMA_SD[0]);
	std::normal_distribution<> dist1(sdot, SIGMA_SD[1]);
	std::normal_distribution<> dist2(s2dot, SIGMA_SD[2]);
	std::normal_distribution<> dist3(d, SIGMA_SD[3]);
	std::normal_distribution<> dist4(ddot, SIGMA_SD[4]);
	std::normal_distribution<> dist5(d2dot, SIGMA_SD[5]);
	return VehicleState(dist0(e2), dist1(e2), dist2(e2), dist3(e2), dist4(e2), dist5(e2), t);
}

Trajectory::Trajectory(vector<double> start_s, vector<double> start_d, VehicleState& goal) {
	vector<double> s_goal = { goal.s, goal.sdot, goal.s2dot };
	vector<double> d_goal = { goal.d, goal.ddot, goal.d2dot };
	vector<double> scoeff = JMT(start_s, s_goal, goal.t);
	vector<double> dcoeff = JMT(start_d, d_goal, goal.t);
	vals = { scoeff[0], scoeff[1], scoeff[2], dcoeff[0], dcoeff[1], dcoeff[2], goal.t };
}

/* PTG: Polynomial Trajectory Generator
 Finds the best trajectory according to WEIGHTED_COST_FUNCTIONS(global).

arguments:
start_s - [s, s_dot, s_ddot]

start_d - [d, d_dot, d_ddot]

target_vehicle - id of leading vehicle(int) which can be used to retrieve
that vehicle from the "predictions" dictionary.This is the vehicle that
we are setting our trajectory relative to.

delta - a length 6 array indicating the offset we are aiming for between us
and the target_vehicle.So if at time 5 the target vehicle will be at
[100, 10, 0, 0, 0, 0] and delta is[-10, 0, 0, 4, 0, 0], then our goal
state for t = 5 will be[90, 10, 0, 4, 0, 0].This would correspond to a
goal of "follow 10 meters behind and 4 meters to the right of target vehicle"

T - the desired time at which we will be at the goal(relative to now as t = 0)

predictions - dictionary of{ v_id: vehicle }. Each vehicle has a method
vehicle.state_in(time) which returns a length 6 array giving that vehicle's
expected[s, s_dot, s_ddot, d, d_dot, d_ddot] state at that time.

return:
(best_s, best_d, best_t) where best_s are the 6 coefficients representing s(t)
best_d gives coefficients for d(t) and best_t gives duration associated w /
this trajectory.
*/

Trajectory Trajectory::Generate(vector<double> start_s, vector<double> start_d, int target_vehicle, vector<double> delta, double T, const map<int, VehicleState>& predictions)
{
    auto it = predictions.find(target_vehicle);
    assert(it != predictions.end());

    const VehicleState& target = it->second;

    // generate alternative goals
    vector<VehicleState> goals;
    double timestep = 0.5;
    double t = T - 4 * timestep;
    while (t <= T + 4 * timestep)
    {
        VehicleState target_state = target.state_in(t).offset(delta);
        goals.emplace_back(target_state);

        for (int i = 0; i < N_SAMPLES; i++)
        {
            goals.emplace_back(target_state.perturb());
        }
        t += timestep;
    }

    // find best trajectory
    vector< Trajectory > trajectories;
    for (int i = 0; i < goals.size(); i++) {
        trajectories.emplace_back(start_s, start_d, goals[i]);
    }

    Trajectory best;
    double min_cost = std::numeric_limits<double>::infinity();
    for (int i = 0; i < trajectories.size(); i++)
    {
        const Trajectory& tr = trajectories[i];
        double c = calculate_cost(tr, target_vehicle, delta, T, predictions);
        if (c < min_cost) {
            min_cost = c;
            best = tr;
        }
    }

    return std::move(best);
}

// -------------------------------------
HighwayState::HighwayState()
{
    state = KL;
	lane = 1;
	ref_vel = 0.0;
}

vector<StateType> HighwayState::successor_states()
{
	vector<StateType> rv;
	rv.push_back(KL);
	// ...
	return std::move(rv);
}

void HighwayState::set_optimal_speed(double v)
{
    v = std::min(v, OPTIMAL_SPEED);
    v = std::max(v, 0.0);

    if (v > ref_vel)
    {
        if (v - ref_vel > .1)
        {
            ref_vel += .1;
        }
        else ref_vel = v;
    }
    else 
    {
		if (ref_vel - v > .1)
		{
			ref_vel -= .1;
		}
		else ref_vel = v;
    }
}

// Car_end_s,card_end_d are the position of our car at the end of the earlier planned route (after car_end_dt sec).
Trajectory HighwayState::advance(double car_end_s, double car_end_d, double car_speed, double car_end_dt, const map<int, VehicleState>& predictions)
{
    assert(car_speed >= 0);
    assert(car_speed < SPEED_LIMIT);
    assert(car_end_d >= 0);
    assert(car_end_d < LANE_WIDTH * LANE_COUNT);
    assert(car_end_s >= 0);
    assert(car_end_dt >= 0);

    // get next state
    vector<StateType> next_state = successor_states();

	int vehicle_front_of_us = -1;
	double min_dist = std::numeric_limits<double>::infinity();
	for (const auto& it : predictions)
	{
		VehicleState v = it.second.state_in(car_end_dt);

		if (v.d<(2 + 4 * lane + 2) && v.d>(2 + 4 * lane - 2))           // is the other vehicle in our lane ?            
		{
			if (car_end_s < v.s) {
				double dist = v.s - car_end_s;
				if (dist < min_dist)
				{
					min_dist = dist;
					vehicle_front_of_us = it.first;
				}
			}
		}
	}

	if (vehicle_front_of_us != -1 && min_dist < MIN_DIST_FROM_CAR_IN_FRONT_OF_US)
	{
		set_optimal_speed(ref_vel-.1);					// too close ! 
	}
	else if (vehicle_front_of_us != -1 && predictions.at(vehicle_front_of_us).sdot<ref_vel)
	{
		double vdiff = ref_vel - predictions.at(vehicle_front_of_us).sdot;
		if (min_dist < vdiff * SPEED_ALIGN_MULTIPLER)
		{
			set_optimal_speed(predictions.at(vehicle_front_of_us).sdot - FOLLOWING_SPEED_DIFF);
			lane = 0;
		}
		else 
		{
			// if there's a car ahead of us but far, don't change speed.
//			set_optimal_speed(ref_vel);
			set_optimal_speed(OPTIMAL_SPEED);
		}
	}
    else {
		set_optimal_speed(OPTIMAL_SPEED);
    }

    return {};
}
