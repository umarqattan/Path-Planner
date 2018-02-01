#include <fstream>
#include <math.h>
#include <chrono>
#include <uWS/uWS.h>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "spline.h"
#include "json.hpp"


using namespace std;

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
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

  double closestLen = 100000;
  int closestWaypoint = 0;

  for(int i = 0; i < maps_x.size(); i++)
  {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if(dist < closestLen)
    {
      closestLen = dist;
      closestWaypoint = i;
    }

  }

  return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
}

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if(next_wp == 0)
  {
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

  if(centerToPos <= centerToRef)
  {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for(int i = 0; i < prev_wp; i++)
  {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
  int prev_wp = -1;

  while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
  {
    prev_wp++;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};

}

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

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
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


   double reference_velocity = 0.0;
   int lane = 1;

  h.onMessage([&reference_velocity, &lane, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
 
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

            auto s = hasData(data);

            if (s != "") {
                auto j = json::parse(s);

                string event = j[0].get<string>();

                if (event == "telemetry") {

                    double car_x = j[1]["x"];
                    double car_y = j[1]["y"];
                    double car_s = j[1]["s"];
                    double car_d = j[1]["d"];
                    double car_yaw = j[1]["yaw"];
                    double car_speed = j[1]["speed"];


                    auto previous_path_x = j[1]["previous_path_x"];
                    auto previous_path_y = j[1]["previous_path_y"];
                    double end_path_s = j[1]["end_path_s"];
                    double end_path_d = j[1]["end_path_d"];
                    auto sensor_fusion = j[1]["sensor_fusion"];

                    json msgJson;

                    vector<double> next_x_vals;
                    vector<double> next_y_vals;

                    if (car_speed == 0) {

                        cout << "The Vehicle has stopped in position: " << car_s << endl;
                    }


                    int path_size = 50;
                    int previous_path_size = previous_path_x.size();


                    if (previous_path_size > 0)
                    {
                        car_s = end_path_s;
                    }

                    bool near_front_vehicle = false;
                    bool left_lane_change = (lane > 0);
                    bool right_lane_change = (lane < 2);

                    
                    double front_thresh = 30;
                    double back_thresh = 10;

                    cout << "The Vehicle is currently in lane " << lane << endl;

                    
                    for(int i = 0; i < sensor_fusion.size(); i++) {

                        
                        double vx = sensor_fusion[i][3];
                        double vy = sensor_fusion[i][4];
                        double neighbor_car_s = sensor_fusion[i][5];
                        float d = sensor_fusion[i][6];
                        double neighbor_car_speed = sqrt(vx*vx+vy*vy);
                        neighbor_car_s += ((double)previous_path_size*.02*neighbor_car_speed);

                        bool is_neighbor_in_front = neighbor_car_s > car_s;
                        bool is_there_front_buffer = neighbor_car_s - car_s < front_thresh;
                        bool is_there_back_buffer = neighbor_car_s - car_s < -back_thresh;
                        bool is_there_lane_buffer = (is_neighbor_in_front && is_there_front_buffer) || is_there_back_buffer;

                        int car_lane = floor(d/4.0);

          
                        if (is_there_lane_buffer) {

                            bool is_neighbor_in_same_lane = (car_lane == lane && is_neighbor_in_front);
                            bool is_neighbor_in_left_lane = (car_lane == lane - 1);
                            bool is_neighbor_in_right_lane = (car_lane == lane + 1);
                            
                            if (is_neighbor_in_same_lane) {

                                near_front_vehicle = true;
                                cout << "The Vehicle is getting closer to the front Vehicle in lane " << lane << endl;
                            }

                            
                            else if (is_neighbor_in_left_lane) {

                                left_lane_change = false;
                                cout << "The Vehicle will not change to left lane " << car_lane << endl;

                            }

                            else if (is_neighbor_in_right_lane) {

                                right_lane_change = false;
                                cout << "The Vehicle will not change to right lane " << car_lane << endl;
                            }
                        }
                    }

                    if (near_front_vehicle) {

                        reference_velocity -= 0.22;

                        if (left_lane_change) {

                            lane -= 1;
                            cout << "Changing to left lane" << endl;

                        }

                        else if (right_lane_change) {

                            lane += 1;
                            cout << "Changing to right lane" << endl;

                        }

                    }
                    else if (reference_velocity < 50) 
                    {

                        reference_velocity += 0.22;

                    }

                    
                    int number_of_points = previous_path_size;

                    vector<double> x_pts;
                    vector<double> y_pts;

                    double x_reference_temperence = car_x;
                    double y_reference = car_y;
                    double y_reference_yaw = deg2rad(car_yaw);

                    if(previous_path_size < 2)
                    {

                        double prev_car_x = car_x - cos(car_yaw);
                        double prev_car_y = car_y - sin(car_yaw);

                        x_pts.push_back(prev_car_x);
                        x_pts.push_back(car_x);

                        y_pts.push_back(prev_car_y);
                        y_pts.push_back(car_y);

                    }
                    else
                    {
                        x_reference_temperence = previous_path_x[number_of_points - 1];
                        y_reference = previous_path_y[number_of_points - 1];

                        double x_reference_temperence_previous = previous_path_x[number_of_points - 2];
                        double y_reference_previous = previous_path_y[number_of_points - 2];
                        y_reference_yaw = atan2(y_reference-y_reference_previous,x_reference_temperence-x_reference_temperence_previous);
                        x_pts.push_back(x_reference_temperence_previous);
                        x_pts.push_back(x_reference_temperence);

                        y_pts.push_back(y_reference_previous);
                        y_pts.push_back(y_reference);

                    }

                    /*
                     * These three waypoints will determine the shape of the spline starting at 30m
                     * and adding multiples of 30m.
                     */

                    double spacing = 30.;
                    vector<double> next_waypoint0 = getXY(car_s+spacing, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
                    vector<double> next_waypoint1 = getXY(car_s+spacing*2, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
                    vector<double> next_waypoint2 = getXY(car_s+spacing*3, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

                    x_pts.push_back(next_waypoint0[0]);
                    x_pts.push_back(next_waypoint1[0]);
                    x_pts.push_back(next_waypoint2[0]);

                    y_pts.push_back(next_waypoint0[1]);
                    y_pts.push_back(next_waypoint1[1]);
                    y_pts.push_back(next_waypoint2[1]);


                    /*
                     * Set the points of the car's frame to that of the relative frame of the Vehicle
                     */

                    for (int i = 0; i < x_pts.size(); i++) {
                        double x_translate = x_pts[i] - x_reference_temperence;
                        double y_translate = y_pts[i] - y_reference;

                        x_pts[i] = (x_translate*cos(-y_reference_yaw)-y_translate*sin(-y_reference_yaw));
                        y_pts[i] = (x_translate*sin(-y_reference_yaw)+y_translate*cos(-y_reference_yaw));
                    }


                    // Initialize and set the points to the spline
                    tk::spline s;

                    s.set_points(x_pts, y_pts);

                    for(int i = 0; i < number_of_points; i++)
                    {
                        next_x_vals.push_back(previous_path_x[i]);
                        next_y_vals.push_back(previous_path_y[i]);
                    }

                    double x_target = 30.;
                    double y_target = s(x_target);
                    double distance_target = sqrt((x_target)*(x_target) + (y_target)*(y_target));

                    double x_inc = 0;

                    for (int i = 0; i < path_size - number_of_points; i++) {

                        double N = (distance_target/(.02*reference_velocity/2.24)); 
                        double x_point = x_inc + (x_target)/N;
                        double y_point = s(x_point);

                        x_inc = x_point;

                        double x_reference_temp = x_point;
                        double y_ref = y_point;

                        x_point = (x_reference_temp * cos(y_reference_yaw)-y_ref*sin(y_reference_yaw));
                        y_point = (x_reference_temp * sin(y_reference_yaw)+y_ref*cos(y_reference_yaw));

                        x_point += x_reference_temperence;
                        y_point += y_reference;

                        next_x_vals.push_back(x_point);
                        next_y_vals.push_back(y_point);

                    }


                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;

                    auto msg = "42[\"control\"," + msgJson.dump() + "]";

                    //this_thread::sleep_for(chrono::milliseconds(1000));
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


