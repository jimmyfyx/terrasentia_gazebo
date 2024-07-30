#ifndef TRAJECTORY_GRABBER_H_
#define TRAJECTORY_GRABBER_H_

/** SECTION:
     Local and System Includes
*/
#include "ros_sensor_bridge_core.h"
#include "udp/udp.h"
#include <terra-simulator-common-messages.h>
#include <terra-simulator-types.h>

#include <fstream>
#include <vector>
#include <sstream>
#include <iterator>

#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;
using namespace terra;

// Utility structure for holding the waypoints
struct CoordinatePair
{
	float x;
	float y;
};

// Overloading the operator
std::istream& operator>>(std::istream& is, CoordinatePair& coordinates)
{
	is >> coordinates.x >> coordinates.y;

	return is;
}

class TrajectoryGrabber{
private:

     // Indexing Variables
     int _count;
	int _num_path;
	
     // Calibration Variables
     float lat_offset;
     float lon_offset;

     // ROS Variables
     ros::NodeHandle m_nh;
     ros::Rate* _loop_rate;
	vector<ros::Subscriber> arrSubs;

     // UDP Communication Variables
     UDP* udp_sock;
     int _port;
     char* _ip_out;

     // Sensor ROS Callbacks
     void pathCallback(const visualization_msgs::MarkerArray::ConstPtr& msg, const int topic_index);

     std::vector<CoordinatePair> waypoints_;
	float lat_;
	float long_;
	std::string waypoints_file_path_;
	const char* filename_;

public:

     // Contructor/DeConstructor
     TrajectoryGrabber(ros::NodeHandle nh, int port, char* ip);
     ~TrajectoryGrabber();

     // Add a ROS sensor topic to listen to retrieve sensor data
     int addRosSensor(string sensor_name, const char* topic_name, int sensor_type);
     int run();

};


#endif // TRAJECTORY_GRABBER_H_
