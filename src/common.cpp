#include <math.h>
#include <string>
#include <vector>
#include <map>
#include <unordered_map>
#include <random>
#include <sstream>
#include "Eigen-3.3/Eigen/Dense"
#include "common.h"

// for convenience
using std::string;
using std::vector;
using std::map;
using std::max;
using std::min;
using std::stringstream;
using std::unordered_map;
using std::endl;
using Eigen::MatrixXd;
using Eigen::VectorXd;

double speed_difference_cost(HighwayState* hw, StateType next_state, const map<int, VehicleState>& predications)
{
	int ln;
	switch (next_state)
	{
	case KL: return 0;
	case LCL: ln = hw->lane - 1; break;
	case LCR: ln = hw->lane + 1; break;
	default:
		assert(false);
		break;
	}
	double behind_dist;
	int vbehind = hw->get_closest_behind(hw->car_end_s, ln * LANE_WIDTH + LANE_WIDTH / 2, hw->car_end_dt, predications, behind_dist);
	if (vbehind == -1) return 0;
	double behind_speed = predications.at(vbehind).sdot;
	if (hw->car_cur_speed >= behind_speed) return 0;
	double spd_diff = behind_speed - hw->car_cur_speed;
	if (spd_diff > MAX_SPEED_DIFF_AT_CHANGING_LANES) return std::numeric_limits<double>::infinity();
	double cost = 1 - exp(-spd_diff / behind_dist);
	return cost;
}

double lane_stay_cost(HighwayState* hw, StateType next_state, const map<int, VehicleState>& predications)
{
	if (hw->last_lane_changed_frame_cnt != -1)
	{
		if (hw->frame_cnt > hw->last_lane_changed_frame_cnt+frames_per_sec * MIN_DURATION_BETWEEN_TWO_LANE_CHANGES)
		{
			// Timer expired, we can change lane from now on.
			hw->last_lane_changed_frame_cnt = -1;
			return 0;
		}
		// Timer is active: We must stay in KL state 
		return next_state == KL ? 0 : std::numeric_limits<double>::infinity();
	}
	// There's no timer, we can leave this lane anytime.
	return 0;
}

double lane_speed_cost(HighwayState* hw, StateType next_state, const map<int, VehicleState>& predications)
{
	int ln;
	switch (next_state) 
	{
	case KL: ln = hw->lane; break;
	case LCL: ln = hw->lane-1; break;
	case LCR: ln = hw->lane+1; break;
	default:
		assert(false);				
		break;
	}
	double ahead_dist, behind_dist;
	int vahead = hw->get_closest_ahead(hw->car_end_s, ln * LANE_WIDTH + LANE_WIDTH / 2, hw->car_end_dt, predications, ahead_dist);
	if (next_state!=KL && vahead != -1 && ahead_dist < SAFE_DISTANCE_FOR_LANE_CHANGE_AHEAD)
	{
		return std::numeric_limits<double>::infinity();
	}
	int vbehind = hw->get_closest_behind(hw->car_end_s, ln * LANE_WIDTH + LANE_WIDTH / 2, hw->car_end_dt, predications, behind_dist);
	if (next_state != KL && vbehind != -1 && behind_dist < SAFE_DISTANCE_FOR_LANE_CHANGE_BEHIND)
	{
		return std::numeric_limits<double>::infinity();
	}

	if (vahead == -1) return 0;				// very good
	double lnspeed = predications.at(vahead).sdot;
/*	if (vbehind != -1)
	{
		lnspeed = min( lnspeed, predications.at(vbehind).sdot );
	} */
	double cost = exp(-lnspeed/10);
	return cost;
}

double space_ahead_cost(HighwayState* hw, StateType next_state, const map<int, VehicleState>& predications)
{
	int ln;
	switch (next_state)
	{
	case KL: ln = hw->lane; break;
	case LCL: ln = hw->lane - 1; break;
	case LCR: ln = hw->lane + 1; break;
	default:
		assert(false);
		break;
	}
	double ahead_dist;
	int vahead = hw->get_closest_ahead(hw->car_end_s, ln * LANE_WIDTH + LANE_WIDTH / 2, hw->car_end_dt, predications, ahead_dist);
	if (vahead == -1)
	{
		return 0.0;
	}
	double cost = exp(-ahead_dist / 100);
	return cost;
}

double convenience_cost(HighwayState* hwstate, StateType next_state, const map<int, VehicleState>& predications)
{
	if (next_state != KL) return CONVENIENCE_COST;
	return 0.0;
}

vector<std::pair< std::function<double(HighwayState* hwstate, StateType next_state, const map<int, VehicleState>& predications)>, double>> WEIGHTED_COST_FUNCTIONS = {
	{ lane_stay_cost, 1 },				// 0 or infinity cost function
	{ lane_speed_cost, 3 },
	{ convenience_cost, 1 },
	{ space_ahead_cost, 2.0 },
	{ speed_difference_cost, 0.5 }
};

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

// -------------------------------------
HighwayState::HighwayState()
{
    state = KL;
	lane = 1;
	ref_vel = 0.0;
#ifdef USE_LOGGING
	logfile.open("log.txt");
#endif 
}

HighwayState::~HighwayState()
{
#ifdef USE_LOGGING
	logfile << "---END---" << endl;
	logfile.close();
#endif
}

const char* HighwayState::state_name(StateType st)
{
	static char buf[512];
	sprintf(buf, "%s", st == KL ? "KL" : (st == LCR ? "LCR" : "LCL"));
	return buf;
}

const char* HighwayState::str(StateType st)
{
	static char buf[512];
	sprintf(buf, "%s (ln: %d, s: %lf, d :%lf v: %lf)", state_name(st), lane, car_cur_s, car_cur_d, car_cur_speed);
	return buf;
}

const char* HighwayState::str_frame()
{
	static char buf[512];
	int secs = int(frame_cnt / frames_per_sec);
	int mins = int(secs / 60);
	int hours = int(mins / 60);
	sprintf(buf, "%d:%d:%d", hours, mins%60, secs-mins*60);
	return buf;
}

double HighwayState::calculate_cost(StateType next_state, const map<int, VehicleState>& predictions, std::string *logstr)
{
	stringstream ss;
	double cost = 0.0;
	bool bInf = false;
	std::string infvector;
	ss << str_frame() << " [" << str(next_state) << "] ";
	int i = 0;
	for (auto& it : WEIGHTED_COST_FUNCTIONS) {
		double new_cost = it.first(this, next_state, predictions);
		if (new_cost == std::numeric_limits<double>::infinity())
		{
			bInf = true;
			infvector += ('1' + i);
		}
		else {
			infvector += ' ';
		}
		ss << new_cost << " ";
		cost += it.second * new_cost;
		i++;
	}
	if (!bInf)
	{
		ss << "--> " << cost << endl;
		if (logfile.is_open()) logfile << ss.str();
	}
#ifdef USE_LOGGING
	if (logstr)
	{
		char buf[512];
		sprintf(buf, "%s [%s] - ", state_name(next_state), infvector.c_str());
		*logstr += buf;
	}
	if (logfile.is_open()) logfile << state_name(next_state) << " [" << infvector.c_str() << "]" << endl;
#endif 
	return cost;
}

vector<StateType> HighwayState::successor_states()
{
	vector<StateType> rv;
	rv.push_back(KL);
	if (state == KL)
	{
		if (lane > 0) rv.push_back(LCL);
		if (lane < LANE_COUNT-1) rv.push_back(LCR);
	}
	return std::move(rv);
}

const double MAX_ACCELERATION = 0.15;

void HighwayState::set_optimal_speed(double v)
{
    v = std::min(v, OPTIMAL_SPEED);
    v = std::max(v, 0.0);

    if (v > ref_vel)
    {
        if (v - ref_vel > MAX_ACCELERATION)
        {
            ref_vel += MAX_ACCELERATION;
        }
        else ref_vel = v;
    }
    else 
    {
		if (ref_vel - v > MAX_ACCELERATION)
		{
			ref_vel -= MAX_ACCELERATION;
		}
		else ref_vel = v;
    }
}

int HighwayState::get_closest_ahead(double s, double d, double dt, const map<int, VehicleState>& predictions, double& min_dist)
{
	int vehicle_id = -1;
	min_dist = std::numeric_limits<double>::infinity();
	for (const auto& it : predictions)
	{
		VehicleState v = it.second.state_in(dt);

		if (v.d<(d + 2) && v.d>(d - 2))           // is the other vehicle in our lane ?            
		{
			if (s < v.s) {
				double dist = v.s - s;
				if (dist < min_dist)
				{
					min_dist = dist;
					vehicle_id = it.first;
				}
			}
		}
	}
	return vehicle_id;
}

int HighwayState::get_closest_behind(double s, double d, double dt, const map<int, VehicleState>& predictions, double& min_dist)
{
	int vehicle_id = -1;
	min_dist = std::numeric_limits<double>::infinity();
	for (const auto& it : predictions)
	{
		VehicleState v = it.second.state_in(dt);

		if (v.d<(d + 2) && v.d>(d - 2))           // is the other vehicle in our lane ?            
		{
			if (s > v.s) {
				double dist = s - v.s;
				if (dist < min_dist)
				{
					min_dist = dist;
					vehicle_id = it.first;
				}
			}
		}
	}
	return vehicle_id;
}

void HighwayState::emergency(const VehicleState& veh)
{
	// clear previous path
	int new_size = std::min((int)prev_path_x.size(), 2);
	prev_path_x.erase(prev_path_x.begin() + new_size, prev_path_x.end());
	prev_path_y.erase(prev_path_y.begin() + new_size, prev_path_y.end());

	// our planned path's end position is the car position with dt=0
	car_end_s = car_cur_s;
	car_end_d = car_cur_d;
	car_end_dt = 0.0;
	ref_vel = car_cur_speed;						// to make it smooth ....
	last_lane_changed_frame_cnt = frame_cnt;		// stay in that lane for a time
	lane = int(car_cur_d / LANE_WIDTH);				// stay in the lane where we are at the moment
	set_optimal_speed(veh.sdot * 0.75);					
}

// Car_end_s,card_end_d are the position of our car at the end of the earlier planned route (after car_end_dt sec).
void HighwayState::advance(const map<int, VehicleState>& predictions)
{
    //assert(car_speed >= 0);
    //assert(car_speed < SPEED_LIMIT);
    //assert(car_end_d >= 0);
    //assert(car_end_d < LANE_WIDTH * LANE_COUNT);
    //assert(car_end_s >= 0);
    //assert(car_end_dt >= 0);

	double dist=0;
	int vid = get_closest_ahead(car_cur_s, lane * LANE_WIDTH + LANE_WIDTH / 2, 0, predictions, dist);
	double slimit = car_end_s - car_cur_s;
	if (vid != -1)
	{
		const VehicleState& veh = predictions.at(vid);
		if (veh.s < car_cur_s + slimit && veh.s > car_cur_s - car_cur_s)
		{
			// emergency: a vehicle is on our preplanned path!

			if (logfile.is_open())
			{
				logfile << str_frame() << " Emergency ! Clearing previously planned path and replanning (st: " << str(state) << ")." << endl;
				printf("Emergency: there is a vehicle in my preplanned path.\n");
			}

			emergency(veh);
		}
	}

	// get next state
    vector<StateType> next_states = successor_states();
	StateType best;
	double min_cost = std::numeric_limits<double>::infinity();
#ifdef USE_LOGGING
	static std::string prevlogstr;
#endif
	std::string logstr;
	for (int i = 0; i < next_states.size(); i++)
	{
		double c = calculate_cost(next_states[i], predictions, &logstr);
		if (c < min_cost) {
			min_cost = c;
			best = next_states[i];
		}
	}

#ifdef USE_LOGGING
	if (prevlogstr.compare(logstr) != 0)
	{
		printf("[%s] %s\n", str_frame(), logstr.c_str());
		prevlogstr = logstr;
	}
#endif // _DEBUG

	assert(min_cost != std::numeric_limits<double>::infinity());
	if (logfile.is_open())
	{
		if (best != state)
		{
			switch (best)
			{
			case KL: logfile << str_frame() << " -> KL" << endl; break;
			case LCL: logfile << str_frame() << " -> LCL" << endl; break;
			case LCR: logfile << str_frame() << " -> LCR" << endl; break;
			default:
				assert(false);
				break;
			}
		}
	}

	switch (best)
	{
	case KL: break;
	case LCL: lane = lane - 1; break;
	case LCR: lane = lane + 1; break;
	default:
		assert(false);
		break;
	}
	state = best;

	if (state == LCL || state == LCR) {
		last_lane_changed_frame_cnt = frame_cnt;
	}

	double min_dist;
	int vehicle_front_of_us = get_closest_ahead(car_end_s, lane * LANE_WIDTH + LANE_WIDTH / 2, car_end_dt, predictions, min_dist);

	if (vehicle_front_of_us != -1 && min_dist < MIN_DIST_FROM_CAR_IN_FRONT_OF_US)
	{
		const VehicleState &veh = predictions.at(vehicle_front_of_us);
		if (last_lane_changed_frame_cnt != -1 && min_dist < SAFE_DISTANCE_FOR_LANE_CHANGE_AHEAD/2)
		{
			if (logfile.is_open())
			{
				logfile << str_frame() << " Emergency ! After lane changing an other vehicle will be dangerously close." << endl;
				printf("Emergency after lane changed.\n");
			}
			emergency(veh);
		}
		else {
			set_optimal_speed(veh.sdot*0.9);					// too close !
		}
	}
	else if (vehicle_front_of_us != -1 && predictions.at(vehicle_front_of_us).sdot < ref_vel)
	{
		double vdiff = ref_vel - predictions.at(vehicle_front_of_us).sdot;
		if (min_dist < vdiff * SPEED_ALIGN_MULTIPLER)
		{
			set_optimal_speed(predictions.at(vehicle_front_of_us).sdot - FOLLOWING_SPEED_DIFF);
		}
		else
		{
			// if there's a car ahead of us but far away, use optimal speed.
			set_optimal_speed(OPTIMAL_SPEED);
		}
	}
	else {
		set_optimal_speed(OPTIMAL_SPEED);
	}

	frame_cnt += 50 - long(round(car_end_dt * 50));
}
