#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

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

double distance(double x1, double y1, double x2, double y2)	{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)	{

	double closestLen = 100000; //large number
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

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)	{

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
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)	{
	
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
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)	{
	
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
  
  double ref_vel=0;
  int lane=1; // start in middle lane			
  double ref_vel_max=49.5; //mph
  ref_vel_max=ref_vel_max*0.44704; //mph to mps
  double max_accel=0.2; // m/s2
  int lane_to_change=0;//initial lane chane to left
  

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&ref_vel,&lane,&ref_vel_max,&max_accel,&lane_to_change](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
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

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;
			
			int prev_path_size=previous_path_x.size();
			
			// Locate our car on the road
			
			if (prev_path_size>0)	{			
				double car_s=end_path_s;			
			}
			
			bool car_ahead=false; //set a boolean 'car ahead' flag to be false initially
			bool lane_change_OK=true; // assuming by default a lane change is OK unless told otherwise
			
			
			// Check location of other cars on the road, especially ones in our lane
			
			for (int i=0;i<sensor_fusion.size();i++)	{
				
				double other_car_d=sensor_fusion[i][6]; // get the 'd' coordinate of the car
				
				if (other_car_d<=4+(4*lane) && other_car_d>=(4*lane))	{ // assuming our car is in center of lane, check + - 2 m in given lane
					
					double other_car_vx=sensor_fusion[i][3];
					double other_car_vy=sensor_fusion[i][4];
					double other_car_speed=sqrt(other_car_vx*other_car_vx+other_car_vy*other_car_vy);
					
					double other_car_s=sensor_fusion[i][5];
					
					other_car_s=other_car_s+ prev_path_size*0.02*other_car_speed; // predict where the car will be at the end of its current planned path
					
					if ((other_car_s > car_s) && (other_car_s-car_s<30)) {					
						car_ahead=true;
					}					
				}			
			}
			
			// decide on which lane you want to shift
			switch (lane) {			
				case 0:
					lane_to_change=1;
					break;
				case 1:
					lane_to_change=0;
					break;
				case 2:
					lane_to_change=1;
					break;
				default:
					cout<<"problem in lane variable"<<endl;			
			}			
			
			if (car_ahead==true) {
			
				cout << "Car Ahead in lane!! " << endl;
				cout << "Possible Lane Change is " << lane_to_change<<endl;
				
				ref_vel=ref_vel-(6.0*max_accel);	// first reduce speed
				ref_vel=max(20*0.44704,ref_vel); // maintain at least 20 mph. If car comes to a full stop, simulator behaves weird
			
				for (int i=0;i<sensor_fusion.size();i++) { // check for lane change
					
					double other_car_d=sensor_fusion[i][6];					
					
					if (other_car_d<=4+(4*lane_to_change) && other_car_d>=(4*lane_to_change)) { // just a simple check that says 'yes - ther			
						
						double other_car_vx=sensor_fusion[i][3];
						double other_car_vy=sensor_fusion[i][4];
						double other_car_speed=sqrt(other_car_vx*other_car_vx+other_car_vy*other_car_vy);
						
						double other_car_s=sensor_fusion[i][5];
						
						other_car_s=other_car_s+ prev_path_size*0.02*other_car_speed; // predict where the car will be at the end of its current planned path
						
						if ((abs(other_car_s-car_s)<20)) { 						
							lane_change_OK=false;
							cout << "Car too close in desired lane change !! " << endl;							
						}
					}
				}
				
				if (lane_change_OK==true && car_speed<35 && car_speed>=30) { 
					lane=lane_to_change;
					cout<<"Changing lane to lane "<<lane<<endl;					
					cout<<"Velocity Condition satisfied for lane change "<<car_speed<<endl;					
				}
			}
			else if ((car_ahead==false)&&(ref_vel<ref_vel_max-0.2)) {		
				ref_vel=ref_vel+max_accel;
			}
			

          	// The iteration runs every 20 milliseconds
			// STEP1: Generate a set of anchor points
			
			vector<double> ptsx;
			vector<double> ptsy;
			
			double ref_car_x=car_x;
			double ref_car_y=car_y;
			double ref_car_yaw=deg2rad(car_yaw);			
			
			if (prev_path_size<2)	{
			
				double prev_car_x=car_x-cos(car_yaw);
				double prev_car_y=car_y-sin(car_yaw);
				
				ptsx.push_back(prev_car_x);
				ptsx.push_back(car_x);
				
				ptsy.push_back(prev_car_y);
				ptsy.push_back(car_y);			
			
			}			
			else	{
			
				ref_car_x=previous_path_x[prev_path_size-1];
				ref_car_y=previous_path_y[prev_path_size-1];
				
				double prev_ref_car_x=previous_path_x[prev_path_size-2];
				double prev_ref_car_y=previous_path_y[prev_path_size-2];
				
				ref_car_yaw=atan2(ref_car_y-prev_ref_car_y,ref_car_x-prev_ref_car_x);
			
				ptsx.push_back(prev_ref_car_x);
				ptsx.push_back(ref_car_x);
				
				ptsy.push_back(prev_ref_car_y);
				ptsy.push_back(ref_car_y);				
			}
			
			
			vector<double> next_wp0=getXY(car_s+30,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y); // vector of {x,y}
			vector<double> next_wp1=getXY(car_s+60,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y); // vector of {x,y}
			vector<double> next_wp2=getXY(car_s+90,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y); // vector of {x,y}
			
			
			ptsx.push_back(next_wp0[0]);
			ptsx.push_back(next_wp1[0]);
			ptsx.push_back(next_wp2[0]);
			
			ptsy.push_back(next_wp0[1]);
			ptsy.push_back(next_wp1[1]);
			ptsy.push_back(next_wp2[1]);
			
			for (int i=0;i<ptsx.size();i++)	{
				
				double shift_x=ptsx[i]-ref_car_x;
				double shift_y=ptsy[i]-ref_car_y;
				
				ptsx[i]=shift_x*cos(0-ref_car_yaw)-shift_y*sin(0-ref_car_yaw);
				ptsy[i]=shift_x*sin(0-ref_car_yaw)+shift_y*cos(0-ref_car_yaw);	
			
			}
			
			//STEP2: Create a spline using anchor points
			
			tk::spline s;
			s.set_points(ptsx,ptsy);
			
			//STEP3: Use the spline to create finer (x,y) points
			
			vector<double> next_x_vals;
			vector<double> next_y_vals;
			
			for (int i=0;i<prev_path_size;i++)	{ // add left over points from previous path that were not executed
				
				next_x_vals.push_back(previous_path_x[i]);
				next_y_vals.push_back(previous_path_y[i]);			
			}
			
			// STEP4: Control vehicle speed by giving target location for vehicle
			
			double target_x=30.0; // arbitrary distance to move
			double target_y=s(target_x); // spline function gives y location for 'x' move
			double target_distance=sqrt(target_x*target_x+target_y*target_y); // hypotenuse of a right triangle
			
			double x_add_on=0;
			
			for (int i=1; i<=50-prev_path_size;i++)	{
			
				double N=target_distance/(ref_vel*0.02); // N=d/vel
				double x_point=x_add_on+(target_x/N); // 0+30/N
				double y_point=s(x_point); // spline give corresponding 'y'
				
				x_add_on=x_point;
				
				double x_ref=x_point;
				double y_ref=y_point;
				
				x_point=x_ref*cos(ref_car_yaw)-y_ref*sin(ref_car_yaw);
				y_point=x_ref*sin(ref_car_yaw)+y_ref*cos(ref_car_yaw);
				
				x_point=x_point+ref_car_x;
				y_point=y_point+ref_car_y;
				
				next_x_vals.push_back(x_point);
				next_y_vals.push_back(y_point);
			
			}			
			
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);          
        }
      }
	  else {
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
