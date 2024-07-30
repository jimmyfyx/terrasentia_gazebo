#ifndef TERRA_ROS_PARAMS_HH
#define TERRA_ROS_PARAMS_HH

#include "params.h"

/** SECTION:
     Typedefs for shorter ROS-type calls
*/
typedef sensor_msgs::Imu imu_t;
typedef nav_msgs::Odometry odom_t;
typedef sensor_msgs::NavSatFix gps_t;
typedef sensor_msgs::Image img_t;
typedef sensor_msgs::LaserScan laser_t;
typedef geometry_msgs::Vector3 vec3_t;


/** SECTION:
     SPECIFIC-TYPE SENSOR INFORMATION
*/
typedef struct ROS_DATA_STAMP{
     float time;
     int seq;
     string frame;
} ROS_DATA_STAMP;

typedef struct ROS_IMU_DATA{
     ROS_DATA_STAMP stamp;
     XYZ_DATA linear;
     XYZ_DATA angular;
     ORIENTATION_DATA orientation;
     XYZ_BASE offset;
} ROS_IMU_DATA;

typedef struct ROS_ODOMETRY_DATA{
     ROS_DATA_STAMP stamp;
     XYZ_DATA position;
     ORIENTATION_DATA orientation;
} ROS_ODOMETRY_DATA;

typedef struct ROS_GPS_DATA{
     ROS_DATA_STAMP stamp;
     float latitude;
     float longitude;
     float altitude;
     XYZ_BASE velocity;
} ROS_GPS_DATA;

typedef struct ROS_LIDAR_DATA{
     ROS_DATA_STAMP stamp;
     float angle_min;
     float angle_max;
     float dAngle;
     float scan_time;
     float dTime;
     float range_min;
     float range_max;
     //float ranges[];
     std::vector<float> ranges;
} ROS_LIDAR_DATA;

typedef struct ROS_CAM_DATA{
     ROS_DATA_STAMP stamp;
     cv::Mat image;
} ROS_CAM_DATA;

/** SECTION:
     MAIN SENSOR INFORMATION STRUCTURE FOR ARRAYS
*/
typedef struct ROS_SENSOR_DATA{
     string sensor_name;
     string topic_name;

     ROS_IMU_DATA imu;
     ROS_ODOMETRY_DATA odom;
     ROS_GPS_DATA gps;
     ROS_LIDAR_DATA lidar;
     ROS_CAM_DATA rgb;
} ROS_SENSOR_DATA;


#endif // TERRA_ROS_PARAMS_HH
