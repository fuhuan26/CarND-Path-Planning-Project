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

int main() {
  uWS::Hub h;

  PID pid;
  
  //twiddle parameters
  bool twiddle = false;
  double p[3] = {0.1,0.001,1};
  double dp[3] ={0.01,0.001,0.1};
  int n = 0;
  int max_n = 400;
  double best_error = 10000.0;
  double error = 0.0;
  double tol = 0.001;
  int p_iterator = 0;
  int sub_move = 0;
  bool first = true;
  bool second = true;
  double best_p[3] = {p[0],p[1],p[2]};
  /**
   * TODO: Initialize the pid variable.
   */

  if (twiddle == true)
  {
    pid.Init(p[0],p[1],p[2]);
  }
  else
  {
    pid.Init(0.121,0.001,1.1);
  }
  
  h.onMessage([&pid,&p,&dp,&n,&max_n,&best_error,&error,&tol,&twiddle,&p_iterator,&sub_move,&first,&second,&best_p](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;

          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */

          
          if (twiddle == false)
          {
          	pid.UpdateError(cte);
          
          	double total_cte = pid.TotalError();
          
          	steer_value = pid.Output();


          // DEBUG
          
          	std::cout << "CTE: " << cte << " cte total: " << total_cte 
                    << std::endl;
          	std::cout << "speed: " << speed << " Steering Tgt: " << steer_value << " Steering Act: " << angle 
                    << std::endl;
          }
          
          else  //twiddle
          {
            error = error + cte*cte;  //unnormalized error
            if(n==0){
              pid.Init(p[0],p[1],p[2]); 
            }
            pid.UpdateError(cte);
            steer_value = pid.Output();
            
            n = n+1;
            if (n > max_n)
            { 
               if(first == true) 
               {
                 p[p_iterator] += dp[p_iterator];
                 first = false;
               }
              else
              {
              
                if (error/max_n < best_error && second == true)
                {
                  best_error = error/max_n;
                  best_p[0] = p[0];
                  best_p[1] = p[1];
                  best_p[2] = p[2];
                  dp[p_iterator] *= 1.1;
                  sub_move += 1;
                }
                else
                {
                  if(second == true) 
                  {
                    p[p_iterator] -= 2 * dp[p_iterator];
                    second = false;
                  }
                  else
                  {
                    if(error/max_n < best_error) 
                    {
                        best_error = error/max_n;
                        best_p[0] = p[0];
                        best_p[1] = p[1];
                        best_p[2] = p[2];
                        dp[p_iterator] *= 1.1;
                        sub_move += 1;
                    }
                    else
                    {
                        p[p_iterator] += dp[p_iterator];
                        dp[p_iterator] *= 0.9;
                        sub_move += 1;
                    }
                  }
                }
              }
              
              if(sub_move > 0) 
              {
                p_iterator = p_iterator+1;
                first = true;
                second = true;
                sub_move = 0;
              }
              
              if (p_iterator == 3)
              {
                p_iterator = 0;
              }
              
              error = 0.0;
              n = 0;
              
              double sumdp = dp[0]+dp[1]+dp[2];
              
              std::cout << "--------------------------------------------------------------------------------------"<<std::endl;
              std::cout << "Best p[0] p[1] p[2]: " << best_p[0] <<" , " << best_p[1] << " , " << best_p[2] <<std::endl;
              std::cout << "sumdp: " << sumdp<<std::endl;
              std::cout << "--------------------------------------------------------------------------------------"<<std::endl;
              if(sumdp < tol) 
              //if(sumdp < 10) 
              {
                std::cout <<"twiddle done!!!!!"<<std::endl;
                ws.close();
                std::cout << "Disconnected" << std::endl;
              } 
              else 
              {
                std::string reset_msg = "42[\"reset\",{}]";
                ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
              }
            }
          }

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = (0.5-std::abs(steer_value));
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
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