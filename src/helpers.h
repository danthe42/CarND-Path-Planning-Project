#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>
#include <map>
#include <unordered_map>
#include <random>
#include "Eigen-3.3/Eigen/Dense"

// for convenience
using std::string;
using std::vector;
using std::map;
using std::unordered_map;
using Eigen::MatrixXd;
using Eigen::VectorXd;

const int N_SAMPLES = 10;
const double SIGMA_SD[6] = { 10.0, 4.0, 2.0, 1.0, 1.0, 1.0 };

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, 
                    const vector<double> &maps_y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, 
                 const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = std::min(2*pi() - angle, angle);

  if (angle > pi()/2) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, 
                         const vector<double> &maps_x, 
                         const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  size_t prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, 
                     const vector<double> &maps_x, 
                     const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
                         (maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}

vector<double> JMT(vector<double>& start, vector<double>& end, double T) {
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

struct VehicleState {
    double s, sdot, s2dot;
    double d, ddot, d2dot;
    VehicleState(const VehicleState& vs) : s(vs.s), sdot(vs.sdot), s2dot(vs.s2dot), d(vs.d), ddot(vs.ddot), d2dot(vs.d2dot) {};
    VehicleState(double _s, double _sdot, double _s2dot, double _d, double _ddot, double _d2dot)
        : s(_s), sdot(_sdot), s2dot(_s2dot)
        , d(_d), ddot(_ddot), d2dot(_d2dot)
    {}

    // predict where should the car be at time T
    VehicleState state_in(double T) const {
        return VehicleState(
            s + (sdot * T) + (s2dot * T * T / 2.0),
            sdot + s2dot * T,
            s2dot,
            d + (ddot * T) + (d2dot * T * T / 2.0),
            ddot + d2dot * T,
            d2dot
        );
    };

    VehicleState offset(vector<double> delta) const
    {
        return VehicleState(s + delta[0], sdot + delta[1], s2dot + delta[2], d + delta[3], ddot + delta[4], d2dot + delta[5]);
    }

    VehicleState perturb_goal() const 
    {
		std::random_device rd;
		std::mt19937 e2(rd());
        std::normal_distribution<> dist0(s, SIGMA_SD[0]);
		std::normal_distribution<> dist1(sdot, SIGMA_SD[1]);
		std::normal_distribution<> dist2(s2dot, SIGMA_SD[2]);
		std::normal_distribution<> dist3(d, SIGMA_SD[3]);
		std::normal_distribution<> dist4(ddot, SIGMA_SD[4]);
		std::normal_distribution<> dist5(d2dot, SIGMA_SD[5]);
        return VehicleState(dist0(e2), dist1(e2), dist2(e2), dist3(e2), dist4(e2), dist5(e2));
    }
};

struct VehicleStateWithT : public VehicleState
{
    double T;
    VehicleStateWithT(const VehicleState& vs, double _T) : VehicleState(vs), T(_T) {} 
};

struct Trajectory {
	vector<double> vals;
	Trajectory(vector<double> s_coeffs, vector<double> d_coeffs, double t) {
		vals = { s_coeffs[0], s_coeffs[1], s_coeffs[2], d_coeffs[0], d_coeffs[1], d_coeffs[2], t };
	}
	Trajectory() {
		vals = { 0,0,0,0,0,0,0 };
	}
    Trajectory(vector<double> start_s, vector<double> start_d, VehicleStateWithT& goal) {
		vector<double> s_goal = { goal.s, goal.sdot, goal.s2dot };
		vector<double> d_goal = { goal.d, goal.ddot, goal.d2dot };
        vector<double> scoeff = JMT(start_s, s_goal, goal.T);
        vector<double> dcoeff = JMT(start_d, d_goal, goal.T);
        vals = { scoeff[0], scoeff[1], scoeff[2], dcoeff[0], dcoeff[1], dcoeff[2], goal.T };
    }
};

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

Trajectory PTG(vector<double> start_s, vector<double> start_d, int target_vehicle, vector<double> delta, double T, const map<int, VehicleState> &predictions )
{
    auto it = predictions.find(target_vehicle);
    assert(it != predictions.end());

    const VehicleState &target = it->second;

    // generate alternative goals
	vector<VehicleStateWithT> goals;
    double timestep = 0.5;
    double t = T - 4 * timestep;
    while (t <= T + 4 * timestep)
    {
        VehicleState target_state = target.state_in(t).offset(delta);
        goals.emplace_back(target_state, t);

        for (int i = 0; i < N_SAMPLES; i++)
        {
            goals.emplace_back(target_state.perturb_goal(), t);
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
#endif  // HELPERS_H