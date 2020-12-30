
#ifdef UWS_VCPKG
    #include <uwebsockets/App.h>
#else
    #include <uWS/uWS.h>
#endif 

#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "common.h"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using std::min;

HighwayState* gHighwayState = nullptr;

// Load up map values for waypoint's x,y,s and d normalized normal vectors
vector<double> map_waypoints_x;
vector<double> map_waypoints_y;
vector<double> map_waypoints_s;
vector<double> map_waypoints_dx;
vector<double> map_waypoints_dy;

// Waypoint map to read from
string map_file_ = "../data/highway_map.csv";

// The max s value before wrapping around the track back to 0
double max_s = 6945.554;

// Process JSON message and generate reply 
std::string process_message(const char* data, size_t length)
{
	std::string msg;
	if (length && length > 2 && data[0] == '4' && data[1] == '2') {

		auto s = hasData(data);

		if (s != "") {
			auto j = json::parse(s);

			string event = j[0].get<string>();

			if (event == "telemetry") {
				// j[1] is the data JSON object

				// Main car's localization Data
				double car_x = j[1]["x"];
				double car_y = j[1]["y"];
				double car_s = j[1]["s"];
				double car_d = j[1]["d"];
				double car_yaw = j[1]["yaw"];
				double car_speed = j[1]["speed"];

				// save the car's current state 
				if (gHighwayState)
				{
					gHighwayState->car_cur_s = car_s;
					gHighwayState->car_cur_d = car_d;
					gHighwayState->car_cur_yaw = car_yaw;
					gHighwayState->car_cur_speed = mph2mps(car_speed);

				}

				// Previous path data given to the Planner
				auto previous_path_x = j[1]["previous_path_x"];
				auto previous_path_y = j[1]["previous_path_y"];

				// Previous path's end s and d values 
				double end_path_s = j[1]["end_path_s"];
				double end_path_d = j[1]["end_path_d"];

				// Sensor Fusion Data, a list of all other cars on the same side 
				//   of the road.
				auto sensor_fusion = j[1]["sensor_fusion"];

				vector<double> next_x_vals;
				vector<double> next_y_vals;
				vector<double> carpos = getFrenet(car_x, car_y, car_yaw, map_waypoints_x, map_waypoints_y);
				map<int, VehicleState> predictions;

				// create other vehicles' state in our format (VehicleState)
				for (auto& it : sensor_fusion)
				{
					double vx = it[3];
					double vy = it[4];
					double speed = sqrt(vx * vx + vy * vy);
					// assume the other cars are going straight in their lanes  (ddot=0)
					predictions.emplace(std::pair<int,VehicleState>(it[0], VehicleState(it[5], speed, 0, it[6], 0, 0)));
				}

				size_t prev_size = previous_path_x.size();

				if (prev_size > 0)
				{
					car_s = end_path_s;
					car_d = end_path_d;
				}

				if (gHighwayState)
				{
					// save the remaining, previously planned path
					gHighwayState->prev_path_x.clear();
					gHighwayState->prev_path_y.clear();
					for (int i = 0; i < prev_size; i++)
					{
						gHighwayState->prev_path_x.push_back(previous_path_x[i]);
						gHighwayState->prev_path_y.push_back(previous_path_y[i]);
					}
					gHighwayState->car_end_s = car_s;
					gHighwayState->car_end_d = car_d;
					gHighwayState->car_end_dt = (double)prev_size * 0.02;


					// Call our logic function.
					// This will handle all the driving logic, by using 
					// - predictions (other vehicles)
					// - From gHighwayState: car_end_s,car_end_d,car_end_dt, prev_path_x, prev_path_y, car_cur_s, car_cur_d, car_cur_yaw, car_cur_speed
					// And the result will contain the new path in gHighwayState attributes: 
					// - prev_path_x, prev_path_y  (As these could be modified !)
					// - lane, ref_vel (These will be used during generation of the new road points on a spline.
					gHighwayState->advance(predictions);

					// use the new "prev_path", it could have been changed !  (@see emergency_mode)
					prev_size = gHighwayState->prev_path_x.size();
					vector<double>& new_previous_path_x = gHighwayState->prev_path_x;
					vector<double>& new_previous_path_y = gHighwayState->prev_path_y;

					vector<double> ptsx, ptsy;
					double ref_x = car_x;
					double ref_y = car_y;
					double ref_yaw = deg2rad(car_yaw);

					if (prev_size < 2)
					{
						// If we have no previous path, use the car's current position, and try to calculate backward using its direction. 
						double prev_car_x = car_x - cos(ref_yaw);
						double prev_car_y = car_y - sin(ref_yaw);
						ptsx.push_back(prev_car_x);
						ptsx.push_back(car_x);
						ptsy.push_back(prev_car_y);
						ptsy.push_back(car_y);
					}
					else
					{
						bool bAddPrevPoint = true;
						ref_x = previous_path_x[prev_size - 1];
						ref_y = previous_path_y[prev_size - 1];
						double ref_x_prev = previous_path_x[prev_size - 2];
						double ref_y_prev = previous_path_y[prev_size - 2];
						ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

						if (ref_x == ref_x_prev && abs(ref_yaw) < 0.0001) {
							// spline generator doesn't like when dx=0, so, avoid this case
							ref_x_prev = new_previous_path_x[0];
							ref_y_prev = new_previous_path_y[0];
							ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
							if (ref_x == ref_x_prev && abs(ref_yaw) < 0.0001) {
								bAddPrevPoint = false;
								ref_yaw = deg2rad(car_yaw);                         // fallback: use the yaw value provided by the simulator
								double prev_car_x = car_x - cos(ref_yaw);
								double prev_car_y = car_y - sin(ref_yaw);
								ptsx.push_back(prev_car_x);
								ptsx.push_back(car_x);
								ptsy.push_back(prev_car_y);
								ptsy.push_back(car_y);
							}
						}

						if (bAddPrevPoint)
						{
							ptsx.push_back(ref_x_prev);
							ptsy.push_back(ref_y_prev);
							ptsx.push_back(ref_x);
							ptsy.push_back(ref_y);
						}
					}

					// Find 3 points on our planned path in 30,60 and 90 meters ahead.
					vector<double> next_wp0 = getXY(car_s + 30, 2 + 4 * gHighwayState->lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
					vector<double> next_wp1 = getXY(car_s + 60, 2 + 4 * gHighwayState->lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
					vector<double> next_wp2 = getXY(car_s + 90, 2 + 4 * gHighwayState->lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

					ptsx.push_back(next_wp0[0]);
					ptsx.push_back(next_wp1[0]);
					ptsx.push_back(next_wp2[0]);
					ptsy.push_back(next_wp0[1]);
					ptsy.push_back(next_wp1[1]);
					ptsy.push_back(next_wp2[1]);

					// Transform our coordinates, so the car position (or the end of our path) gets to the origin, and rotate it so the path gets almost horizontal. That's necessary for the spline generator. 
					// Later we will transform them back.
					for (int i = 0; i < ptsx.size(); i++)
					{
						double shift_x = ptsx[i] - ref_x;
						double shift_y = ptsy[i] - ref_y;
						ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
						ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
					}

					//Calculate the spline
					tk::spline s;
					s.set_points(ptsx, ptsy);

					// put back the remaining (undriven) previous path
					for (int i = 0; i < prev_size; i++)
					{
						next_x_vals.push_back(new_previous_path_x[i]);
						next_y_vals.push_back(new_previous_path_y[i]);
					}
					double target_x = 30.0;
					double target_y = s(target_x);
					double target_dist = sqrt(target_x * target_x + target_y * target_y);
					double x_add_on = 0;

					// And append the necessary number of points to the end of the list, using our spline
					for (int i = 1; i <= 50 - prev_size; i++)
					{
						double N = (target_dist / (0.02 * gHighwayState->ref_vel));
						double x_point = x_add_on + target_x / N;
						double y_point = s(x_point);
						x_add_on = x_point;

						double x_ref = x_point;
						double y_ref = y_point;

						x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
						y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

						x_point += ref_x;
						y_point += ref_y;

						next_x_vals.push_back(x_point);
						next_y_vals.push_back(y_point);
					}
				}

				json msgJson;
				msgJson["next_x"] = next_x_vals;
				msgJson["next_y"] = next_y_vals;

				msg = "42[\"control\"," + msgJson.dump() + "]";
			}  // end "telemetry" if
		}
		else {
			// Manual driving
			msg = "42[\"manual\",{}]";
		}
	}  // end websocket if
	return msg;
}

#ifndef UWS_VCPKG
int main() {
  uWS::Hub h;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }
	
	if (map_waypoints_x.size()==0)
	{
		printf("Error opening/loading the highway map file: %s", map_file_.c_str());
		exit(-1);	
	}

  h.onMessage([](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
	auto msg = process_message(data, length);
	if (!msg.empty())
	{
		ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
	}
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
	gHighwayState = new HighwayState;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
	if (gHighwayState)
	{
		delete gHighwayState;
		gHighwayState = nullptr;
	}

	std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}

#else 

int main() {

    std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

    string line;
    while (getline(in_map_, line)) {
        std::istringstream iss(line);
        double x;
        double y;
        float s;
        float d_x;
        float d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        map_waypoints_x.push_back(x);
        map_waypoints_y.push_back(y);
        map_waypoints_s.push_back(s);
        map_waypoints_dx.push_back(d_x);
        map_waypoints_dy.push_back(d_y);
    }
	
    struct PerSocketData {

	};

    int port = 4567;

    uWS::App::WebSocketBehavior b;
    b.open = [](auto* ws) {
        std::cout << "Connected!!!" << std::endl;
        gHighwayState = new HighwayState;
    };
    b.close = [](auto* ws, int /*code*/, std::string_view /*message*/) {
        ws->close();
        if (gHighwayState)
        {
            delete gHighwayState;
            gHighwayState = nullptr;
        }

        std::cout << "Disconnected" << std::endl;
    };

	b.message = [] (auto* ws, std::string_view message, uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        size_t length = message.length();
        const char* data = message.data();
        auto msg = process_message(data, length);
		if (!msg.empty())
		{
			ws->send(msg, uWS::OpCode::TEXT);
		}
    };

    uWS::App().ws<PerSocketData>("/*", std::move(b)).listen("127.0.0.1", port, [port](auto* listen_socket) {
        if (listen_socket) {
            std::cout << "Listening on port " << port << std::endl;
        }
        else {
			std::cerr << "Failed to listen to port" << std::endl;
			exit(-1);
        }
    }).run();
}

#endif 
