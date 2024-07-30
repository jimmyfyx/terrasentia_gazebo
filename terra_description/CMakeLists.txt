cmake_minimum_required(VERSION 3.0.2)
project(terra_description)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	geometry_msgs
	visualization_msgs
	message_generation
	cv_bridge
)

find_package(gazebo REQUIRED)
find_package(roslaunch)

link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

#######################################
## Declare ROS messages and services ##
#######################################

add_message_files(
  FILES
  XYZ_Base.msg
  Euler_Base.msg
  Quaternion_Base.msg
  Terra_XYZ_Data.msg
  Terra_Orientation_Data.msg
  TerraImu.msg
  TerraGps.msg
  TerraEncoder.msg
  TerraBattery.msg
  FourWD.msg
  Motor.msg
  FullGPS.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES geometry_msgs visualization_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
	CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
	${catkin_INCLUDE_DIRS}
	${GAZEBO_INCLUDE_DIRS}
)

###################
## Build Plugins ##
###################

add_library(TorsionalSpringPlugin SHARED plugins/TorsionalSpringPlugin.cc)
target_link_libraries(TorsionalSpringPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(CameraGimbalPlugin SHARED plugins/CameraGimbalPlugin.cc)
target_link_libraries(CameraGimbalPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(terra_ros_imu plugins/terra_ros_imu.cpp)
target_link_libraries(terra_ros_imu ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(terra_ros_imu ${PROJECT_NAME}_generate_messages_cpp)

add_library(terra_ros_gps_with_dropout plugins/terra_ros_gps_with_dropout.cpp)
target_link_libraries(terra_ros_gps_with_dropout ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(terra_ros_gps_with_dropout ${PROJECT_NAME}_generate_messages_cpp)

add_library(terra_ros_skid_steer plugins/terra_skid_steering_with_encoder.cpp)
target_link_libraries(terra_ros_skid_steer ${catkin_LIBRARIES})
add_dependencies(terra_ros_skid_steer ${PROJECT_NAME}_generate_messages_cpp)

add_library(terra_ros_encoder plugins/terra_encoder.cpp)
target_link_libraries(terra_ros_encoder ${catkin_LIBRARIES})
add_dependencies(terra_ros_encoder ${PROJECT_NAME}_generate_messages_cpp)
