#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "MPC.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    std::cout << sdata << std::endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          //////////////////////////////////////////////////////////////////////
          // Get the updated data
          //   - ref path, state of vehicle, and control inputs.
          //   - j[1] is the data JSON object
          //////////////////////////////////////////////////////////////////////
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
		  double steer_value = j[1]["steering_angle"];
          double throttle_value = j[1]["throttle"];

          //////////////////////////////////////////////////////////////////////
          // Calculate steering angle and throttle using MPC.
          //   - Both are in between [-1, 1].
          //////////////////////////////////////////////////////////////////////

          //********************************************************************
          // transform waypoints from global coordinate to car's coordinate
          //********************************************************************
		  size_t n_waypoints = ptsx.size();
          auto ptsx_transformed = Eigen::VectorXd(n_waypoints);
          auto ptsy_transformed = Eigen::VectorXd(n_waypoints);
          for (unsigned int i = 0; i < n_waypoints; i++ ) {
            double dX = ptsx[i] - px;
            double dY = ptsy[i] - py;
            double minus_psi = 0.0 - psi;
            ptsx_transformed( i ) = dX * cos( minus_psi ) - dY * sin( minus_psi );
            ptsy_transformed( i ) = dX * sin( minus_psi ) + dY * cos( minus_psi );
          }

          // Fit polynomial to the points - 3rd order.
          auto coeffs = polyfit(ptsx_transformed, ptsy_transformed, 3);
          
		  //********************************************************************
          // Get current error estimates
          // - cte:
          //   The cross track error is calculated by evaluating at polynomial at x, f(x)
          //   and subtracting y.
          // - epsi:
          //   Current heading error epsi is the tangent to the road curve at x
          //   epsi = arctan(f') where f' is the derivative of the fitted polynomial
          //   f' = coeffs[1] + 2.0*coeffs[2]*x + 3.0*coeffs[3]*x*x
          //********************************************************************
		  double cte = polyeval(coeffs, 0);  // px = 0, py = 0
          double epsi = - atan(coeffs[1]);  // p

          //********************************************************************
          // Get the delayed state as the new initial state
          //********************************************************************
		  
		  // Center of gravity needed related to psi and epsi
          const double Lf = 2.67;
		  // Latency for predicting time at actuation
          const double latency = 0.1;

          // Predict state after latency using the kinematic model
          // x, y and psi are all zero after transformation above
          double pred_px = 0.0 + v * latency; // Since psi is zero, cos(0) = 1, can leave out
          double pred_py = 0.0; // Since sin(0) = 0, y stays as 0 (y + v * 0 * latency)
          double pred_psi = 0.0 + v * -steer_value / Lf * latency; // change of sign because turning left is negative in simulator but positive for MPC
          double pred_v = v + throttle_value * latency;
          double pred_cte = cte + v * sin(epsi) * latency;
          double pred_epsi = epsi + v * -steer_value / Lf * latency;

		  Eigen::VectorXd state(6);
          state << pred_px, pred_py, pred_psi, pred_v, pred_cte, pred_epsi;
          
		  //********************************************************************
          // Solve for new actuations
          // And to show predicted x and y in the future
          // Using Model Predictive Control
          //********************************************************************
		  auto vars = mpc.Solve(state, coeffs);

		  //********************************************************************
          // Construct the message to the simulater:
          //   1) Control input: steering, throttle
          //   2) Predicted path points to be displayed
          //********************************************************************

		  // Update Actuation Values
		  // Steering must be divided by deg2rad(25) to normalize within [-1, 1].
  		  // Multiplying by Lf takes into account vehicle's turning ability
          steer_value = vars[0] / (deg2rad(25) * Lf);
          throttle_value = vars[1];

		  // Send Actuation Messages
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //********************************************************************
          // Display the MPC predicted trajectory
          //********************************************************************
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          /**
           *   add (x,y) points to list here, points are in reference to 
           *   the vehicle's coordinate system the points in the simulator are 
           *   connected by a Green line
           */
		  for (int i = 2; i < vars.size(); i ++) {
            if (i%2 == 0) {
              mpc_x_vals.push_back(vars[i]);
            }
            else {
              mpc_y_vals.push_back(vars[i]);
            }
          }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //********************************************************************
          // Display the waypoints/reference line
          //********************************************************************
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           *   add (x,y) points to list here, points are in reference to 
           *   the vehicle's coordinate system the points in the simulator are 
           *   connected by a Yellow line
           */
		  for (double i = 0; i < 100; i += 3){
            next_x_vals.push_back(i);
            next_y_vals.push_back(polyeval(coeffs, i));
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

		  //********************************************************************
          // Construct the message to be passed to the simulator / car
          //********************************************************************
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;

          // Latency
          // The purpose is to mimic real driving conditions where
          //   the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          //   around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE SUBMITTING.
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          
		  //********************************************************************
          // send the message to the simulator via WebSocket
          //********************************************************************
		  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
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