#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

#define INIT_KP             0.15
#define INIT_KI             0
#define INIT_KD             1.35
#define TWIDDLE_ITERATIONS  500
#define TOL                 0.5
#define MAX_THROTTLE        0.3
#define MIN_THROTTLE        0.3

#define DP                  0.1
#define DI                  0.0001
#define DD                  0.1

int main()
{
  uWS::Hub h;

  PID pid;
  PID throttle_pid;

  int iteration = 0;
  double err = 0;
  double best_err = -1;
  double dp[3] = { DP, DI, DD };
  double p[3] = { INIT_KP, INIT_KI, INIT_KD };
  int   index = 0;
  int   state = 0;

  // Initialize the pid variable.
  pid.Init(0.15, 0.0001, 1.45548);

  throttle_pid.Init(0.472162, 0.000130564, 0.164954);

#ifdef UWS_VCPKG
  h.onMessage([&pid, &throttle_pid, &iteration, &err, &best_err, &dp, &p, &index, &state](uWS::WebSocket<uWS::SERVER> *ws, char *data, size_t length, uWS::OpCode opCode) {
#else
  h.onMessage([&pid, &throttle_pid, &iteration, &err, &best_err, &dp, &p, &index, &state](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
#endif
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value, throttle_value;
          /*
          * Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          pid.UpdateError(cte);
          steer_value = pid.TotalError();

          throttle_pid.UpdateError(fabs(steer_value));
          throttle_value = MIN_THROTTLE;
          if (MAX_THROTTLE - fabs(throttle_pid.TotalError()) > throttle_value) {
            throttle_value = MAX_THROTTLE - fabs(throttle_pid.TotalError());
          }

          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
          std::cout << "Iteration: " << iteration << " Err: " << err << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
#ifdef UWS_VCPKG
          ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#else
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#endif

          if (dp[0] + dp[1] + dp[2] > TOL) {
            iteration++;
            if (iteration >= TWIDDLE_ITERATIONS) {
              err += cte * cte;
            }
            if (iteration == TWIDDLE_ITERATIONS * 2) {
              err = err / TWIDDLE_ITERATIONS;
              if (best_err < 0) { // the best_err has not setup yet
                best_err = err;
                index = 0;
                state = 0;
              }
              else {
                if (state == 0) {
                  if (err < best_err) {
                    dp[index] *= 1.1;
                    best_err = err;
                  }
                  else {
                    p[index] -= 2 * dp[index];
                    state = 1;
                  }
                }
                else {
                  if (err < best_err) {
                    best_err = err;
                  }
                  else {
                    p[index] += dp[index];
                    dp[index] *= 0.9;
                  }
                  state = 0;
                }

                if (state == 0) {
                  index = (index + 1) % 3;
                }
              }

              std::cout << "parameters: " << p[0] << "," << p[1] << "," << p[2] << std::endl;
              std::cout << "dps: " << dp[0] << "," << dp[1] << "," << dp[2] << std::endl;
              std::cout << "best_err: " << best_err << std::endl;
              std::cout << "index: " << index << " state: " << state << std::endl;

              msg = "42[\"reset\",{}]";
#ifdef UWS_VCPKG
              ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#else
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#endif
              iteration = 0;
              err = 0;
              if (state == 0) {
                p[index] += dp[index];
              }
              pid.Init(p[0], p[1], p[2]);
            }
          }
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
#ifdef UWS_VCPKG
        ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#else
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#endif
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

#ifdef UWS_VCPKG
  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> *ws, uWS::HttpRequest req) {
#else
  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
#endif
    std::cout << "Connected!!!" << std::endl;
  });

#ifdef UWS_VCPKG
  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> *ws, int code, char *message, size_t length) {
    ws->close();
#else
  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
#endif
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen("127.0.0.1", port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
