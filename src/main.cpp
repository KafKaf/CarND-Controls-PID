#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <valarray>

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

void reset_simulator(uWS::WebSocket<uWS::SERVER>& ws) {
  std::string msg("42[\"reset\", {}]");
  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
  std::cout << "Simulator Reset!!!" << std::endl;
}

// Implementation of twiddle algorithm with the simulator
bool twiddle_update(PID &pid, std::valarray<double> &p, std::valarray<double> &dp, double &tolerance, double &best_err
, int &p_index, int &level, double err) {

//  // faster runs on the simulator since integral response is best with 0.
//  if (p_index == 1)
//    p_index == 2;

  if (dp.sum() > tolerance) {
    if (level == 0) {
      p[p_index] += dp[p_index];
      pid.Init(p[0], p[1], p[2]);

      // twiddle adjustment for the simulator
      level = 1;
    } else if (level == 1) {
      if (err < best_err) {
        // DEBUG
        std::cout << "new best error = " << err << std::endl;
        std::cout << "new best p = " << p[0] << ", " << p[1] << ", " << p[2] << std::endl;
        std::cout << "new best dp = " << dp[0] << ", " << dp[1] << ", " << dp[2] << std::endl;
        std::cout << "p_index = " << p_index << ", level: " << level << std::endl;
        best_err = err;
        dp[p_index] *= 1.1;

        // return to first step - twiddle adjustment for the simulator
        level = 0;
        p_index = (p_index + 1) % p.size();
      } else {
        p[p_index] -= 2 *dp[p_index];
        pid.Init(p[0], p[1], p[2]);

        // twiddle adjustment for the simulator
        level = 2;
      }
    } else if (level == 2) {
     if (err < best_err) {
       // DEBUG
       std::cout << "new best error = " << err << std::endl;
       std::cout << "new best p = " << p[0] << ", " << p[1] << ", " << p[2] << std::endl;
       std::cout << "new best dp = " << dp[0] << ", " << dp[1] << ", " << dp[2] << std::endl;
       std::cout << "p_index = " << p_index << ", level: " << level << std::endl;
       best_err = err;
       dp[p_index] *= 1.1;
     } else {
       p[p_index] += dp[p_index];
       dp[p_index] *= 0.9;
     }

     // return to first step - twiddle adjustment for the simulator
     level = 0;
     p_index = (p_index + 1) % p.size();
    }
      // DEBUG
      std::cout << "err = " << err << std::endl;
      std::cout << "new p = " << p[0] << ", " << p[1] << ", " << p[2] << std::endl;
      std::cout << "new dp = " << dp[0] << ", " << dp[1] << ", " << dp[2] << std::endl;
      std::cout << "new p_index = " << p_index << ", new level: " << level << std::endl;
      return true;
  }

  return false;
}

double moving_average(std::deque<double> &deque, double &value, int &max_size) {
    if (deque.size() >= max_size) {
      deque.pop_front();
    }

    deque.push_back(value);

    double sum = 0;
    for (int i = 0; i < deque.size(); i++) {
      sum += deque[i];
    }

    return sum / double(deque.size());
}

int main(int argc, char* argv[])
{
  // enable twiddle, run like this: ./pid twiddle
  bool twiddle_tuning = false;
  if (argc >= 2) {
    if (argv[1] == std::string("twiddle")) {
      twiddle_tuning = true;
    }
  }

  uWS::Hub h;
  PID pid;

  std::valarray<double> p (0.0,3); // initialize p array with zeros
  std::valarray<double> dp (1.0,3); // initialize dp array with zeros

  // currently best co efficients and error received from twiddle running on the simulator
  // comment out to start from scratch, converge faster if given direction(some good values at start)
  p[0] = 0.466328;
  p[1] = 0;
  p[2] = 2.79457;
  dp[0] = 0.0927142;
  dp[1] = 0.0278128;
  dp[2] = 0.227583;
  double best_err = 0.0485864; // twiddle - best error achieved
  double tolerance = 0.01; // twiddle - tolerance, run until sum(dp) > tolerance
  int p_index = 1; // twiddle - p index to start with
  int level = 0; // twiddle - level
  int current_iteration = 0; // twiddle - current step
  double current_err = 0.0; // twiddle - current error
  int min_iterations = 100; // twiddle - minimum steps for the simulator before reset
  int max_iterations = 600; // twiddle - maximum steps for the simulator before reset
  int half_min_iterations = min_iterations / 2; // twiddle - sum error after this number of steps
  double cte_term_failure = 3.0; // twiddle - reset simulator if average cte is bigger than this
  double throttle = 0.3; // current throttle
  double max_throttle = 0.3; // max throttle
  double min_throttle = 0.05; // min throttle
  double high_steering_angle = 0.15; // slow down on sharp turns, same as humans

//  int queue_max_size = 4; // moving average of size n
//  std::deque<double> deque(queue_max_size, 0);

  // initialize pid controller
  pid.Init(p[0], p[1], p[2]);

  double cte = 0;
  h.onMessage([&pid, &p, &dp, &tolerance, &best_err, &p_index, &level, &current_iteration, &current_err, &min_iterations, &max_iterations
    , &half_min_iterations, &cte, &cte_term_failure, &throttle, &max_throttle, &min_throttle, &twiddle_tuning
    , &high_steering_angle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {

    // only for twiddle tuning
    if (twiddle_tuning) {
      double err = current_err / current_iteration;
      if ((current_iteration > min_iterations && err > cte_term_failure) || current_iteration > max_iterations) {
        reset_simulator(ws);
        current_iteration = 0;
        current_err = 0.0;
        bool should_continue = twiddle_update(pid, p, dp, tolerance, best_err, p_index, level, err);
          if (!should_continue) {
            exit(1);
          }
      }
    }

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
          cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());

          current_iteration++;
          if (current_iteration >= half_min_iterations) {
            current_err += pow(cte, 2);
          }

          // update errors
          pid.UpdateError(cte);

          // get new steering angle and smooth it with the previous steering angle
          double steer_value = pid.TotalError();

//          steer_value = moving_average(deque, steer_value, queue_max_size);

          // set throttle to max
          throttle = max_throttle;

          // slow down on fast turns (ignore with twiddle to optimize it best for high speed)
          if (!twiddle_tuning && fabs(steer_value) >= high_steering_angle) {
            throttle = min_throttle;
          }

          // DEBUG
//          std::cout << "CTE: " << cte << ", Steering Value: " << steer_value << ", Throttle: " << throttle << std::endl;

          // DEBUG
//          std::cout << "p_error: " << pid.p_error << ", i error: " << pid.i_error << ", d_error: " << pid.d_error << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
//          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
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

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
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
