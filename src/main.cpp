#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

/*
 * Function to transform coordinates from map to car's coordinate system
 * psi - car's heading in map coordinates
 * (x_car, y_car) - car's position in map coordinates
 * (x_point, y_point) - point position in map coordinates
 * returns the point's coordinates in car's coordinates
 */
vector<double> map_to_car_coord(double psi, double x_car, double y_car, double x_point, double y_point){
  double car_x = (x_point - x_car) * cos(psi) + (y_point - y_car) * sin(psi);
  double car_y = (y_point - y_car) * cos(psi) - (x_point - x_car) * sin(psi);
  return {car_x, car_y};
}


int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
      // "42" at the start of the message means there's a websocket message event.
      // The 4 signifies a websocket message
      // The 2 signifies a websocket event
      string sdata = string(data).substr(0, length);
      cout << sdata << endl;
      if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
        string s = hasData(sdata);
        if (s != "") {
          auto j = json::parse(s);
          string event = j[0].get<string>();
          if (event == "telemetry") {
            // j[1] is the data JSON object
            vector<double> ptsx = j[1]["ptsx"];
            vector<double> ptsy = j[1]["ptsy"];
            double px = j[1]["x"];
            double py = j[1]["y"];
            double psi = j[1]["psi"];
            double v = j[1]["speed"];
            v *= 0.44704; //convert from mph to m/s

            double steer_value_in = j[1]["steering_angle"];
            steer_value_in *= deg2rad(25);
            double throttle_value_in = j[1]["throttle"];

            /*
           * Calculate steeering angle and throttle using MPC.
           *
           * Both are in between [-1, 1].
           *
           */
            Eigen::VectorXd ptsx_car = Eigen::VectorXd(ptsx.size());
            Eigen::VectorXd ptsy_car = Eigen::VectorXd(ptsx.size());

            // Convert from map coordinates to vehicle coordinates
            for (int i = 0; i < ptsx.size(); i++){
              auto car_coord = map_to_car_coord(psi, px, py, ptsx[i], ptsy[i]);
              ptsx_car[i] = car_coord[0];
              ptsy_car[i] = car_coord[1];

            }

            auto coeffs = polyfit(ptsx_car, ptsy_car, 3);

            // errors in the current car position
            double cte = polyeval(coeffs, 0) - 0.0;
            double epsi = -atan(coeffs[1]);

            // Create current state vector and solve
            // Add a latency of 100ms into the state before sending it to solver
            Eigen::VectorXd state(6);
            double latency = 0.1; //add a latency of 100ms
            double Lf = 2.67;
            double x_dl = (0.0 + v * latency);
            double y_dl = 0.0;
            double psi_dl = 0.0 + v * steer_value_in / Lf * latency;
            double v_dl = 0.0 + v + throttle_value_in * latency;
            double cte_dl = cte + (v * sin(epsi) * latency);
            double epsi_dl = epsi + v * steer_value_in / Lf * latency;

            state << x_dl, y_dl, psi_dl, v_dl, cte_dl, epsi_dl; //latency compensated state

            auto result = mpc.Solve(state, coeffs);

            // Compute steering and angle value
            double steer_value = -result[6]/deg2rad(25); //normalize to [-1, 1]
            double throttle_value = result[7];

            json msgJson;
            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = throttle_value;

            // MPC predicted trajectory
            vector<double> mpc_x_vals = mpc.solution_x_;
            vector<double> mpc_y_vals = mpc.solution_y_;

            // Waypoints line
            vector<double> next_x;
            vector<double> next_y;

            for (int i = 0; i < ptsx.size(); i++){
              //auto car_coord = map_to_car_coord(psi, px, py, ptsx[i], ptsy[i]);
              next_x.push_back(ptsx_car[i]);
              next_y.push_back(ptsy_car[i]);
            }

            //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
            // the points in the simulator are connected by a Yellow line
            msgJson["mpc_x"] = mpc_x_vals;
            msgJson["mpc_y"] = mpc_y_vals;

            msgJson["next_x"] = next_x;
            msgJson["next_y"] = next_y;


            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            std::cout << msg << std::endl;
            // Latency
            // The purpose is to mimic real driving conditions where
            // the car does actuate the commands instantly.
            //
            // Feel free to play around with this value but should be to drive
            // around the track with 100ms latency.
            //
            // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
            // SUBMITTING.
            this_thread::sleep_for(chrono::milliseconds((int)(latency*1000)));
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
        } else {
          // Manual driving
          std::string msg = "42[\"manual\",{}]";
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
      const std::string s = "<h1>Hello world!</h1>";
      if (req.getUrl().valueLength == 1) {
        res->end(s.data(), s.length());
      } else {
        // i guess this should be done more gracefully?
        res->end(nullptr, 0);
      }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
      std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
      ws.close();
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
