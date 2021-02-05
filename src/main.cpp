#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;

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
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}
#define sign(a) (a)>=0? (1):(-1)

int main() {
  uWS::Hub h;

  PID pid_steer;
  PID pid_speed;
  /**
   * TODO: Initialize the pid variable.
   */
  //double Kp_steer=0.13,Kd_steer=.21,Ki_steer=0.002;
  double Kp_steer=0.17,Kd_steer=.5,Ki_steer=0.0002;
  double Kp_speed = 0.07, Kd_speed = 0.01, Ki_speed = 0;
  pid_steer.Init(Kp_steer, Ki_steer, Kd_steer);
  pid_speed.Init(Kp_speed, Ki_speed, Kd_speed);

  h.onMessage([&pid_steer, &pid_speed](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(string(data).substr(0, length));

      if (s != "")
      {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry")
        {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
          double des_speed = 15;
          double throttle_value;
          double speed_err = speed - des_speed;
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
          pid_steer.UpdateError(cte);
          pid_steer.Calculate_Control_Value(steer_value);
          
          pid_speed.UpdateError(speed_err);
          pid_speed.Calculate_Control_Value(throttle_value);

          //DEBUG
          
          std::cout << "1. CTE: " << cte << " Steering Value: " << steer_value << " Steered: " << angle
                    << std::endl;

          std::cout << "2. SppedErr: " << speed_err << " Throttle Value: " << throttle_value
                    << std::endl;

          std::cout << "3. Total Steering Err: " << pid_steer.TotalError() 
                    << std::endl;          

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        } // end "telemetry" if
      }
      else
      {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    } // end websocket message if
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