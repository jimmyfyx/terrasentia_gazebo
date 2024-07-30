#ifndef __CAMERA_GIMBAL_PLUGIN_HH__
#define __CAMERA_GIMBAL_PLUGIN_HH__

#include <string>

#include "gazebo/common/common.hh"
#include "gazebo/common/Events.hh"

#include "gazebo/gazebo.hh"

#include "gazebo/physics/physics.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/Joint.hh"

#include <boost/thread.hpp>
#include <boost/bind.hpp>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>

namespace gazebo{
     class CameraGimbalPlugin : public ModelPlugin{
          private:
               void ExplicitUpdate();
			void QueueThread();

               event::ConnectionPtr updateConnection;
               physics::ModelPtr model;
			physics::JointPtr jointHandle;

			ros::NodeHandle* nh_;

               ros::Subscriber targetAngleSub_;
               ros::Subscriber externalCmdSub_;
			ros::Subscriber imuSub_;
               ros::Publisher curAnglePub_;

			std::string robot_namespace_;
			std::string jointName;
			std::string gimbal_imu_topic_;
			std::string gimbal_cmd_topic_;
			std::string target_angle_topic;
			std::string target_angle_param;

			int useImuAxisFlag;

			double commandOmega;
			double imuDegreeOffset;
			double imuOffset;

			// Physical Angle Limits (radians)
			double maxAngle_;
			double minAngle_;
			// Gimbal Control Limits
			double maxCmd_;
			double minCmd_;
			// PID Integral Error Limits to help prevent "Integral Windup"
			double maxIntegralErr_;
			double minIntegralErr_;

			double imuRoll;
			double imuPitch;
			double imuYaw;
               double trueAngle;

			double lockedJointTruth;
			double lockedRoll;
			double lockedPitch;
			double lockedYaw;

			double kP;
               double kI;
               double kD;

			double control_gain_;		// Used as a adjustable multiplier to the PID calculated control value
			double _target_angle;
			double _integral = 0;
			double prev_error = 0;
			double prev_input = 0;
			double prev_output = 0;

			// Time Specific Variables
			common::Time prevUpdateTime;
			double update_rate_;
			double update_period_;

			// Flags
			bool verbose;
			bool use_offset_;
			bool alive_;
			bool flip_control_direction_;

			// TODO: Can be set by users via URDF I/O, however INTERNAL USAGE HAS NOT BEEN IMPLEMENTED YET
			bool flip_joint_direction_;

			// Miscellaneous Parameters
			double torque_;

			// Callback Queue
			ros::CallbackQueue queue_;
	          boost::thread callback_queue_thread_;

			// Mutex Lock to prevent race-conditions
               boost::mutex lock;

          public:

               CameraGimbalPlugin();
               virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
               virtual void Init();

               void imuCallback(const sensor_msgs::Imu::ConstPtr& cmd_msg);

			double getFeedbackAngle(bool unwrap = true);
			double pidStep(double _sensor_feedback, double _dt);

			void updateImuAngles();
			void setTargetAngle(double angle);

			bool isJointLimitReached(bool enforce_limits = false);

		protected:
			virtual void FiniChild();

	};
}
#endif /** __CAMERA_GIMBAL_PLUGIN_HH__ */
