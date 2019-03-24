#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

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
  
  // Start in Lane 1 (middle lane)
  int lane = 1;
  
  // Move at reference velocity to target
  double ref_vel = 0.0; //mph
  
  // Safe distance to avoid collision
  double safe_distance = 30.0; // metres

  h.onMessage([&ref_vel,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&lane,&safe_distance]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
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

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
          
          int prev_size = previous_path_x.size();
          
          // Use sensor fusion data to avoid collision in frenet co-ordinates
          if (prev_size > 0)
          {
            car_s = end_path_s;
          }
          
          // Preferred Lane Calculation Starts------------------------
          
          // Calculate average speed of lanes
          double right_speed = 0;
          double left_speed = 0;
          double middle_speed = 0;
          
          float right_size = 0;
          float left_size = 0;
          float middle_size = 0;
          bool collision = false;
          
          for (int index = 0; index < sensor_fusion.size(); index++)
          {
            float d_ = sensor_fusion[index][6];
            
            // Get car speed 
            double vx_ = sensor_fusion[index][3];
            double vy_ = sensor_fusion[index][4];
            double check_speed_ = sqrt(vx_*vx_ + vy_*vy_);
            double check_car_s_ = sensor_fusion[index][5];
            
            // Project s value into future
            check_car_s_ = check_car_s_ + ((double)prev_size*0.02*check_speed_);
            if (fabs(check_car_s_ - car_s) < safe_distance)
            {
              collision = true;
            }
            
            // Determine which lane the car is on
            int car_lane_;
            if (d_ > 0 && d_< 4)
            {
              car_lane_ = 0;
            }
            else if (d_ > 4 && d_ < 8)
            {
              car_lane_ = 1;
            }
            else if (d_ > 8 && d_ < 12)
            {
              car_lane_ = 2;
            }
            else
            {
              continue;
            }
            
            // Calculate average speed of lanes
            if (car_lane_ == 0)
            {
              left_speed = left_speed + check_speed_;
              left_size += 1;
            }
            else if (car_lane_ == 1)
            {
              middle_speed = middle_speed + check_speed_;
              middle_size += 1;
            }
            else if (car_lane_ == 2)
            {
              right_speed = right_speed + check_speed_;
              right_size += 1;
            }
          }
          
          double average_left_speed = left_speed/left_size;
          double average_middle_speed = middle_speed/middle_size;
          double average_right_speed = right_speed/right_size;
          
          vector<double> lane_speeds_ = {average_left_speed, average_middle_speed, average_right_speed};
          
          vector<double> costs;
          int min_cost = 9999999;
          double max_speed = 50;
          for (int j = 0; j < 3; j++)
          {
            //double cost_ = inefficiency_cost(max_speed, j, lane_speeds_);
            double cost_ = 1 -  (lane_speeds_[j]/max_speed);
            if (collision)
            {
              cost_ = min_cost;
            }
            costs.push_back(cost_);
          }
          
          int fastest_lane = lane;
   
          for (int k = 0; k < costs.size(); k++)
          {
            if (costs[k] < min_cost)
            {
              min_cost = costs[k];
              fastest_lane = k;
            }
          }
          
          // Preferred Lane Calculation Ends--------------------------
          
          // Car Ahead flag
          bool car_same_lane = false;
          
          // Car on left lane flag
          bool car_left_lane = false;
          
          // Car on right lane flag
          bool car_right_lane = false;
          
          //find reference velocity to use
          for (int i = 0; i < sensor_fusion.size(); i++)
          {
            float d = sensor_fusion[i][6];
            
            // Determine which lane the car is on
            int car_lane;
            if (d > 0 && d< 4)
            {
              car_lane = 0;
            }
            else if (d > 4 && d < 8)
            {
              car_lane = 1;
            }
            else if (d > 8 && d < 12)
            {
              car_lane = 2;
            }
            else
            {
              continue;
            }

            // Get car speed and s value 
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx + vy*vy);
            double check_car_s = sensor_fusion[i][5];
            
            // Project s value into future
            check_car_s = check_car_s + ((double)prev_size*0.02*check_speed);
              
            // Car in my lane
            if(car_lane == lane)
            {
              // Set car ahead flag
              if ((check_car_s > car_s) && ((check_car_s - car_s) < safe_distance))
              {
                car_same_lane = true;
              }
            }
            
            // Car on left lane
            if(car_lane == (lane - 1))
            {
              // Set car on left lane flag
              if (fabs(check_car_s - car_s) < safe_distance)
              {
                car_left_lane = true;
              }
            }
            
            // Car on right lane
            if(car_lane == (lane + 1))
            {
              // Set car on right lane flag
              if ((fabs(check_car_s - car_s)  < safe_distance))
              {
                car_right_lane = true;
              }
            } 
          }
          
          if(car_same_lane)
          {
            if ((!car_left_lane) && (lane > 0))
            {
              lane--;
            }
            else if ((!car_right_lane) && (lane < 2))
            {
              lane++;
            }
            else
            {
              ref_vel = ref_vel - 0.224; // equivalent of deceleration of 5 m/s^2
            }
          }          
          else 
          {
            if ( lane != fastest_lane ) 
            { // if we are not on the fastest lane.
              if ( (fastest_lane > lane) || (!car_right_lane) ) 
              {
                lane++; // Shift right.
              }
              if ( (fastest_lane < lane) || (!car_left_lane) ) 
              {
                lane--; // Shift left.
              }
            }
            if (ref_vel < 49.5) // desired speed = 49.5 mph
            {
              ref_vel = ref_vel + 0.224; // equivalent of acceleration of 5 m/s^2
            }
          }

          json msgJson;

          //vector<double> next_x_vals;
          //vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          
          // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
          // Later interpolate these waypoints with a spline and fill it with more points that control speed 
          vector<double> ptsx;
          vector<double> ptsy;
            
          // Reference x, y, yaw states
          // Reference the start point, where the car is, or the previous path end point
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          
          // If previous size is almost empty, use the car as starting referene
          if(prev_size < 2)
          {
            // use two points that make the path tangent to the car
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          // Use previous path's end points as starting reference
          else
          {
            // redefine the reference points as previous path's end points
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];
            
            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            
            // Use above points that make the path tangent to the previous path end points
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }
          
          // In Frenet co-ordinate, add evenly 30m spaced points ahead of starting reference
          vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
          
          // Transform these added waypoints from global to car co-ordinates so that calculation is easier without any angle
          for (int i = 0; i < ptsx.size(); i++)
          {
            // Shift
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;
            
            // Rotate
            ptsx[i] = shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw);
            ptsy[i] = shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw);
          }
          
          // Create spline
          tk::spline s;
          
          // Set (x,y) points to the spline
          s.set_points(ptsx, ptsy);
          
          // Define actual (x,y) points for planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          // Start with all the previous path points from the last time
          for (int i = 0; i < previous_path_x.size(); i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          // Calculate break up of spline points so that reference velocity is maintained
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x)*(target_x) + (target_y)*(target_y));
          
          double x_add_on = 0;
          
          // Fill up rest of path planner after filing it with previous points. Here, there are always 50 points as ouput
          for (int  i = 0; i <= (50-previous_path_x.size()); i++)
          {
            double N = (target_dist/(0.02*ref_vel/2.24)); // ref_vel is in miles per hour
            double x_point = x_add_on + (target_x)/N;
            double y_point = s(x_point);
            
            x_add_on = x_point;
            
            double x_ref = x_point;
            double y_ref = y_point;
            
            // Transform car co-ordinates back to global
            // Rotate
            x_point = (x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw));
            y_point = (x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw));
            
            // Shift
            x_point = x_point + ref_x;
            y_point = y_point + ref_y;
            
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }
          
          // end

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

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