#ifndef PARAMS_HH
#define PARAMS_HH

using namespace std;
static const int MAX_NUM_SENSORS = 25;

/** SECTION:
     Variables Useful for Identification
*/
// typedef enum SimulatorMessageType{
// 	SIMULATOR_MESSAGE_SET_PORT        = 0,
// 	SIMULATOR_MESSAGE_CONTROL         = 1,
// 	SIMULATOR_MESSAGE_SENSOR_DATA     = 2,
// 	SIMULATOR_MESSAGE_DATA_RCCOMMANDS = 3,
// }SimulatorMessageType;
//
// typedef enum SimulatorDataType{
//      SIMULATOR_DATA_NONE          = 0,
//      SIMULATOR_DATA_IMU           = 1,
//      SIMULATOR_DATA_ENCODER       = 2,
//      SIMULATOR_DATA_GPS           = 3,
//      SIMULATOR_DATA_RGB_CAMERA    = 4,
//      SIMULATOR_DATA_RGBD_CAMERA   = 5,
//      SIMULATOR_DATA_LIDAR         = 6,
//      SIMULATOR_DATA_3D_LIDAR      = 7,
//      SIMULATOR_DATA_3D_CAMERA     = 8,
//      SIMULATOR_DATA_ODOMETRY      = 9,
// }SimulatorDataType;

typedef enum SimulatorMeasurementType{
     SIMULATOR_MEASUREMENT_SINGLE    = 0,
     SIMULATOR_MEASUREMENT_MULTIPLE  = 1,
}SimulatorMeasurementType;

/** SECTION:
     COMMUNICATION PARAMETERS
*/
// typedef struct CommunicationHeaderByte{
//      int32_t header;
//      int32_t msg_type;
//      int32_t data_type;
//      int32_t measurement_type;
//      int32_t measurement_length;
// }CommunicationHeaderByte;

/** SECTION:
     BASE MEASUREMENT DATA
*/
typedef struct XYZ_BASE{
     float x;
     float y;
     float z;
} XYZ_BASE;

typedef struct ORIENTATION_QUATERNION_BASE{
     float x;
     float y;
     float z;
     float w;
} ORIENTATION_QUATERNION_BASE;

typedef struct ORIENTATION_EULER_BASE{
     float roll;
     float pitch;
     float yaw;
} ORIENTATION_EULER_BASE;

/** SECTION:
     EXTRAPOLATED INFORMATION
*/
typedef struct XYZ_DATA{
     float x;
     float y;
     float z;
     XYZ_BASE covariance;
     XYZ_BASE bias;
	XYZ_BASE offset;
} XYZ_DATA;

typedef struct ORIENTATION_DATA{
     float x;
     float y;
     float z;
     float w;
     ORIENTATION_EULER_BASE covariance;
     ORIENTATION_QUATERNION_BASE bias;
} ORIENTATION_DATA;

/** SECTION:
     SENSOR DATA TYPES
*/
// typedef struct Sim_IMUData{
//      XYZ_DATA accel;
//      XYZ_DATA gyro;
//      ORIENTATION_DATA orientation;
// }Sim_IMUData;
//
// typedef struct Sim_EncoderData{
//      int pulses;
//      float speed;
// }Sim_EncoderData;
//
// typedef struct Sim_OdometryData{
//      XYZ_DATA position;
//      ORIENTATION_DATA orientation;
// }Sim_OdometryData;
//
// typedef struct Sim_GPSData{
// 	float time;
// 	float latitude;
// 	float longitude;
// 	float altitude;
// 	XYZ_BASE velocity;
// 	XYZ_BASE covariance;
// 	int32_t covariance_type;
// 	int32_t service;
// 	int32_t status;
// }Sim_GPSData;
//
// typedef struct Sim_LidarData{
//      float angle_min;
//      float angle_max;
//      float dAngle;
//      float scan_time;
//      float dTime;
//      float range_min;
//      float range_max;
// 	vector<float> ranges;
//      vector<float> intensities;
// }Sim_LidarData;


#endif // PARAMS_HH
