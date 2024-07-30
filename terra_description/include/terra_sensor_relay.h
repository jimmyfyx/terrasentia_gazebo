#ifndef TERRA_SENSOR_RELAY_H_
#define TERRA_SENSOR_RELAY_H_

/** SECTION:
     Local and System Includes
*/
#include "ros_sensor_bridge_core.h"
#include "udp/udp.h"
#include <terra-simulator-common-messages.h>
#include <terra-simulator-types.h>
#include <terrasentia_sensors/TerraImu.h>
#include <terrasentia_sensors/TerraEncoder.h>
#include <terrasentia_sensors/TerraGps.h>
#include <terrasentia_sensors/TerraBattery.h>


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

class TerraSensorRelay{

private:

     // Indexing Variables
     int _count;
     int _num_sensors;
     int _num_imu;
     int _num_gps;
     int _num_odom;
     int _num_encoder;
     int _num_rgb;
     int _num_rgbd;
     int _num_lidar;
     int _num_battery;

     // Calibration Variables
     double terra_lat_origin;
     double terra_long_origin;

     // ROS Variables
     ros::NodeHandle m_nh;
     image_transport::ImageTransport _it;
     ros::Rate* _loop_rate;

     // Containers for Sensors and sensor data available
     vector<ros::Subscriber> arrSubs;
     vector<Sim_Msg_IMUData> arrImu;
     vector<Sim_Msg_GPSData> arrGps;
     vector<Sim_Msg_LidarData> arrLidar;
     vector<Sim_Msg_EncoderData> arrEncoder;
     vector<Sim_Msg_BatteryData> arrBattery;

     // UDP Communication Variables
     UDP* udp_sock;
     int _port;
     char* _ip_out;

     // Sensor ROS Callbacks
     void imuCallback(const terrasentia_sensors::TerraImu::ConstPtr& msg, const int topic_index);
     void gpsCallback(const terrasentia_sensors::TerraGps::ConstPtr& msg, const int topic_index);
     void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg, const int topic_index);
     void encoderCallback(const terrasentia_sensors::TerraEncoder::ConstPtr& msg, const int topic_index);
     void batteryCallback(const terrasentia_sensors::TerraBattery::ConstPtr& msg, const int topic_index);
     void pathCallback(const visualization_msgs::MarkerArray::ConstPtr& msg, const int topic_index);

     // TODO
     // vector<Sim_OdometryData> arrOdom;
     // vector<ROS_SENSOR_DATA> arrRgb;
     // vector<ROS_SENSOR_DATA> arrRgbd;

     // void odomCallback(const nav_msgs::Odometry::ConstPtr& msg, const int topic_index);
     // void camCallback(const sensor_msgs::Image::ConstPtr& msg, const int topic_index);

     std::vector<CoordinatePair> waypoints_;
	double lat_;
	double long_;
	std::string waypoints_file_path_;
	const char* filename_;

public:

     // Contructor/DeConstructor
     TerraSensorRelay(ros::NodeHandle nh, int port, char* ip);
     ~TerraSensorRelay();

     // Add a ROS sensor topic to listen to retrieve sensor data
     int addRosSensor(string sensor_name, const char* topic_name, int sensor_type);
     int run();

     // UDP Sending Overloads
     int sendUdp(int _port, char* _add, CommunicationHeaderByte* header, Sim_Msg_IMUData data);
     int sendUdp(int _port, char* _add, CommunicationHeaderByte* header, Sim_Msg_GPSData data);
     int sendUdp(int _port, char* _add, CommunicationHeaderByte* header, Sim_Msg_LidarData data);
     int sendUdp(int _port, char* _add, CommunicationHeaderByte* header, Sim_Msg_EncoderData data);
     int sendUdp(int _port, char* _add, CommunicationHeaderByte* header, Sim_Msg_BatteryData data);

     void printUdpHeader(CommunicationHeaderByte* _header);
     void printImu(Sim_Msg_IMUData data);
     void printGps(Sim_Msg_GPSData data);
     void printLidar(Sim_Msg_LidarData data);
     void printEncoder(Sim_Msg_EncoderData data);
     void printBattery(Sim_Msg_BatteryData data);
};


#endif // TERRA_SENSOR_RELAY_H_
