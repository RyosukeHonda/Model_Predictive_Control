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
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
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
    //cout << sdata << endl;
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

          for (int i = 0; i < ptsx.size(); i++ )
          {
            //rotate the difference between the waypoints and the car position by -psi.
            double diff_x = ptsx[i]-px;
            double diff_y = ptsy[i]-py;
            ptsx[i] = (diff_x *cos(-psi)-diff_y*sin(-psi));
            ptsy[i] = (diff_x *sin(-psi)+diff_y*cos(-psi));
          }

          Eigen::Map<Eigen::VectorXd> ptsx_trans(ptsx.data(), ptsx.size());
          Eigen::Map<Eigen::VectorXd> ptsy_trans(ptsy.data(),ptsy.size());

          auto coeffs = polyfit(ptsx_trans, ptsy_trans, 3);

          //caculate cte and epsi
          //Using car coordinate, (0,0) is the car location at time step t.
          double cte = polyeval(coeffs, 0);
          //we can simplify the epsi by using car coordinate since the position of x is always 0 at time step t.
          //-atan(coeffs[1]+2.0*coeffs[2]*x+3.0*coeffs[3]*x*x -> -atan(coeffs[1]))
          double epsi = -atan(coeffs[1]);

          //Current steer_value and throttle_value
          double steer_value = j[1]["steering_angle"];
          double throttle_value = j[1]["throttle"];

          double delay_t = .1;
          const double Lf = 2.67;

          //transition from t to t+1 (time step is delta_t = 0.1)
          double delay_x = v * delay_t;
          double delay_y = 0;
          double delay_psi = -v * steer_value / Lf * delay_t;
          double delay_v = v + throttle_value * delay_t;
          double delay_cte = cte + v * sin(epsi) * delay_t;
          double delay_epsi = epsi-v * steer_value /Lf * delay_t;


          Eigen::VectorXd state(6);
          state << delay_x, delay_y, delay_psi, delay_v, delay_cte, delay_epsi;

          /*
          * TODO: Calculate steeering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */

          //solve MPC
          auto vars = mpc.Solve(state, coeffs);

          vector<double> next_x_vals;
          vector<double> next_y_vals;

       /* Print center of the road
        * //x increment in car coordinate
          double x_inc = 5;
          //number of points
          int num_points = 15;
          for (int i = 1; i < num_points; i++)
          {
                next_x_vals.push_back(x_inc*i);
                next_y_vals.push_back(polyeval(coeffs, x_inc*i));
          }
        */
            //Print the waypoints(converted to the car coordinate)
            for(int i = 0; i<ptsx.size();i++){
                next_x_vals.push_back(ptsx[i]);
                next_y_vals.push_back(ptsy[i]);
            }
          //hold the prediction of x and y value in car coordinate.
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          //first two values are steer_angle and throttle.
          //The rest is x and y value
          for (int i = 2; i < vars.size(); i++){
          // x prediction
                if(i % 2 == 0){
                    mpc_x_vals.push_back(vars[i]);
                }
          //y prediction
                else{
                    mpc_y_vals.push_back(vars[i]);
                }
          }

          json msgJson;
          msgJson["steering_angle"] = vars[0];
          msgJson["throttle"] = vars[1];

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

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
          this_thread::sleep_for(chrono::milliseconds(100));
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
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
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