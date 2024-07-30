#include <ros/ros.h>
#include <tf/tf.h>
#include <math.h>

#include "gazebo/physics/physics.hh"
#if GAZEBO_MAJOR_VERSION >= 9
	// #include <ignition/math.hh>
	#include <ignition/math/Vector3.hh>
	#include <ignition/math/Quaternion.hh>
#else
	#include <gazebo/math/gzmath.hh>
#endif
#include <geometry_msgs/Quaternion.h>

#include "CameraGimbalPlugin.hh"

#define M_RAD2DEG 360/(2*M_PI)
#define M_DEG2RAD (2*M_PI)/360

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(CameraGimbalPlugin)

/////////////////////////////////////////////////
CameraGimbalPlugin::CameraGimbalPlugin(){
}

/////////////////////////////////////////////////
void CameraGimbalPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){

     this->model = _model;
     this->gimbal_cmd_topic_ = "odom";
     this->gimbal_imu_topic_ = "front_gimbal_imu";
     this->target_angle_param = "front_gimbal_target";
     this->robot_namespace_ = "";

	this->useImuAxisFlag = 1;

	this->imuOffset = 0.0;
	this->imuDegreeOffset = 0.0;
	this->update_rate_ = 50.0;
	this->_target_angle = 0.0;
	this->control_gain_ = 1.0;
	this->torque_ = 5.0;

	this->maxAngle_ = 90.0 * M_DEG2RAD;
	this->minAngle_ = -90.0 * M_DEG2RAD;
	this->maxCmd_ = 1.0;
	this->minCmd_ = -1.0;
	this->maxIntegralErr_ = 0.3;
	this->minIntegralErr_ = -0.3;

	this->alive_ = true;
	this->use_offset_ = true;
	this->flip_control_direction_ = false;
	this->flip_joint_direction_ = false;

    if (!_sdf->HasElement("robotNamespace")) {
      ROS_INFO_NAMED("libCameraGimbalPlugin", "CameraGimbalPlugin Plugin missing <robotNamespace>, defaults to \"%s\"",
          this->robot_namespace_.c_str());
    } else {
      this->robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
    }

	if(!_sdf->HasElement("jointHandle")){
		ROS_INFO_NAMED("libCameraGimbalPlugin", "Plugin missing <jointHandle>, defaults to /base_link");
		this->jointName = "/base_link";
	} else this->jointName = _sdf->Get<std::string>("jointHandle");

	if(!_sdf->HasElement("kP")){
		ROS_INFO_NAMED("libCameraGimbalPlugin", "Plugin missing <kP>, defaults to 100.0");
		this->kP = 100.0;
	} else this->kP = _sdf->Get<double>("kP");

	if(!_sdf->HasElement("kI")){
		ROS_INFO_NAMED("libCameraGimbalPlugin", "Plugin missing <kI>, defaults to 1.0");
		this->kI = 1.0;
	} else this->kI = _sdf->Get<double>("kI");

	if(!_sdf->HasElement("kD")){
		ROS_INFO_NAMED("libCameraGimbalPlugin", "Plugin missing <kD>, defaults to 1.0");
		this->kD = 1.0;
	} else this->kD = _sdf->Get<double>("kD");

	if(!_sdf->HasElement("verbose")){
		ROS_INFO_NAMED("libCameraGimbalPlugin", "Plugin missing <verbose>, defaults to false");
		this->verbose = false;
	} else this->verbose = _sdf->Get<bool>("verbose");


    if (!_sdf->HasElement("gimbalSpeedTopic")) {
         ROS_WARN_NAMED("libCameraGimbalPlugin", "CameraGimbalPlugin Plugin (ns = %s) missing <gimbalSpeedTopic>, defaults to \"%s\"",
          this->robot_namespace_.c_str(), this->gimbal_cmd_topic_.c_str());
    } else {
      this->gimbal_cmd_topic_ = _sdf->GetElement("gimbalSpeedTopic")->Get<std::string>();
    }

    if(!_sdf->HasElement("imuTopic")){
         ROS_WARN_NAMED("libCameraGimbalPlugin", "CameraGimbalPlugin Plugin (ns = %s) missing <imuTopic>, defaults to \"%s\"",
          this->robot_namespace_.c_str(), this->gimbal_imu_topic_.c_str());
    } else {
      this->gimbal_imu_topic_ = _sdf->GetElement("imuTopic")->Get<std::string>();
    }

    if(!_sdf->HasElement("useImuAxis")){
         ROS_WARN_NAMED("libCameraGimbalPlugin", "CameraGimbalPlugin Plugin (ns = %s) missing <useImuAxis>, defaults to \"%s\". This parameter is used to determine which axis on the IMU is used as feedback for the PID controller ([NOTE] Acceptable values are the following: 1 - Roll, 2 - Pitch, 3 - Yaw).",
          this->robot_namespace_.c_str(), this->useImuAxisFlag);
    } else {
      this->useImuAxisFlag = _sdf->GetElement("useImuAxis")->Get<int>();
	 // TODO: Properly handle input error cases here
    }

    if(!_sdf->HasElement("imuDegreeOffset")){
         ROS_WARN_NAMED("libCameraGimbalPlugin", "CameraGimbalPlugin Plugin (ns = %s) missing <imuDegreeOffset>, defaults to \"%s\"",
          this->robot_namespace_.c_str(), this->imuDegreeOffset);
    } else {
      this->imuDegreeOffset = _sdf->GetElement("imuDegreeOffset")->Get<double>();
	 this->imuOffset = this->imuDegreeOffset * M_DEG2RAD;
    }

    if(!_sdf->HasElement("updateRate")){
	    ROS_WARN_NAMED("libCameraGimbalPlugin", "CameraGimbalPlugin Plugin (ns = %s) missing <updateRate>, defaults to \"%s\"",
		this->robot_namespace_.c_str(), this->update_rate_);
    } else {
	 this->update_rate_ = _sdf->GetElement("updateRate")->Get<double>();
    }

	// Initialize update rate stuff
	if(this->update_rate_ > 0.0){ this->update_period_ = 1.0 / this->update_rate_;}
	else{ this->update_period_ = 0.0;}


	if(!_sdf->HasElement("useOffset")){
 	    ROS_WARN_NAMED("libCameraGimbalPlugin", "CameraGimbalPlugin Plugin (ns = %s) missing <useOffset>, defaults to \"%s\"",
 		this->robot_namespace_.c_str(), this->use_offset_);
     } else {
 	 this->use_offset_ = _sdf->GetElement("useOffset")->Get<bool>();
     }

	if(!_sdf->HasElement("defaultTargetAngle")){
 	    ROS_WARN_NAMED("libCameraGimbalPlugin", "CameraGimbalPlugin Plugin (ns = %s) missing <defaultTargetAngle>, defaults to \"%s\"",
 		this->robot_namespace_.c_str(), this->_target_angle);
     } else {
 	 this->_target_angle = _sdf->GetElement("defaultTargetAngle")->Get<double>();
	 this->setTargetAngle(this->_target_angle);
     }

	if(!_sdf->HasElement("maxJointAngle")){
		ROS_WARN_NAMED("libCameraGimbalPlugin",
			"CameraGimbalPlugin Plugin (ns = %s) missing <maxJointAngle>, defaults to \"%s\"",
			this->robot_namespace_.c_str(), this->maxAngle_ * M_RAD2DEG);
     } else{
		this->maxAngle_ = _sdf->GetElement("maxJointAngle")->Get<double>() * M_DEG2RAD;
     }

	if(!_sdf->HasElement("minJointAngle")){
		ROS_WARN_NAMED("libCameraGimbalPlugin",
			"CameraGimbalPlugin Plugin (ns = %s) missing <minJointAngle>, defaults to \"%s\"",
			this->robot_namespace_.c_str(), this->minAngle_ * M_RAD2DEG);
     } else{
		this->minAngle_ = _sdf->GetElement("minJointAngle")->Get<double>() * M_DEG2RAD;
     }

	if(!_sdf->HasElement("maxCommandSignal")){
		ROS_WARN_NAMED("libCameraGimbalPlugin",
			"CameraGimbalPlugin Plugin (ns = %s) missing <maxCommandSignal>, defaults to \"%s\"",
			this->robot_namespace_.c_str(), this->maxCmd_);
     } else{
		this->maxCmd_ = _sdf->GetElement("maxCommandSignal")->Get<double>();
     }

	if(!_sdf->HasElement("minCommandSignal")){
		ROS_WARN_NAMED("libCameraGimbalPlugin",
			"CameraGimbalPlugin Plugin (ns = %s) missing <minCommandSignal>, defaults to \"%s\"",
			this->robot_namespace_.c_str(), this->minCmd_);
     } else{
		this->minCmd_ = _sdf->GetElement("minCommandSignal")->Get<double>();
     }

	if(!_sdf->HasElement("maxIntegralErr")){
		ROS_WARN_NAMED("libCameraGimbalPlugin",
			"CameraGimbalPlugin Plugin (ns = %s) missing <maxIntegralErr>, defaults to \"%s\"",
			this->robot_namespace_.c_str(), this->maxIntegralErr_);
     } else{
		this->maxIntegralErr_ = _sdf->GetElement("maxIntegralErr")->Get<double>();
     }

	if(!_sdf->HasElement("minIntegralErr")){
		ROS_WARN_NAMED("libCameraGimbalPlugin",
			"CameraGimbalPlugin Plugin (ns = %s) missing <minIntegralErr>, defaults to \"%s\"",
			this->robot_namespace_.c_str(), this->minIntegralErr_);
     } else{
		this->minIntegralErr_ = _sdf->GetElement("minIntegralErr")->Get<double>();
     }

	if(!_sdf->HasElement("controlGain")){
		ROS_WARN_NAMED("libCameraGimbalPlugin",
		"CameraGimbalPlugin Plugin (ns = %s) missing <controlGain>, defaults to \"%s\"",
		this->robot_namespace_.c_str(), this->control_gain_);
	} else{
		this->control_gain_ = _sdf->GetElement("controlGain")->Get<double>();
	}

	if(!_sdf->HasElement("flipControlDirection")){
		ROS_WARN_NAMED("libCameraGimbalPlugin",
		"CameraGimbalPlugin Plugin (ns = %s) missing <flipControlDirection>, defaults to \"%s\"",
		this->robot_namespace_.c_str(), this->flip_control_direction_);
	} else{
		this->flip_control_direction_ = _sdf->GetElement("flipControlDirection")->Get<bool>();
	}

	if(!_sdf->HasElement("flipJointAngleDirection")){
		ROS_WARN_NAMED("libCameraGimbalPlugin",
		"CameraGimbalPlugin Plugin (ns = %s) missing <flipJointAngleDirection>, defaults to \"%s\"",
		this->robot_namespace_.c_str(), this->flip_joint_direction_);
	} else{
		this->flip_joint_direction_ = _sdf->GetElement("flipJointAngleDirection")->Get<bool>();
	}

	if(!_sdf->HasElement("torque")){
		ROS_WARN_NAMED("libCameraGimbalPlugin",
		"CameraGimbalPlugin Plugin (ns = %s) missing <torque>, defaults to \"%s\"",
		this->robot_namespace_.c_str(), this->torque_);
	} else{
		this->torque_ = _sdf->GetElement("torque")->Get<double>();
	}

	if(!_sdf->HasElement("targetAngleParamName")){
		ROS_WARN_NAMED("libCameraGimbalPlugin",
		"CameraGimbalPlugin Plugin (ns = %s) missing <targetAngleParamName>, defaults to \"%s\"",
		this->robot_namespace_.c_str(), this->target_angle_param);
	} else{
		this->target_angle_param = _sdf->GetElement("targetAngleParamName")->Get<std::string>();
	}


}

/////////////////////////////////////////////////
void CameraGimbalPlugin::Init(){

	// Prepare Joint stuff
	this->jointHandle = this->model->GetJoint(this->jointName);
     this->jointHandle->SetParam("fmax", 0, this->torque_);

     ignition::math::Vector3d jointAx = this->jointHandle->LocalAxis(0);

     // Flip axis of joint rotation if needed
     if(this->flip_joint_direction_){
          jointAx = -1 * jointAx;
          this->jointHandle->SetAxis(0, jointAx);
     }

	// ROS Setup
	nh_ = new ros::NodeHandle(this->robot_namespace_);

	// Subscribe to the Gimbal's IMU sensor for PID feedback
     ros::SubscribeOptions so = ros::SubscribeOptions::create<sensor_msgs::Imu>
		(gimbal_imu_topic_, 1, boost::bind(&CameraGimbalPlugin::imuCallback, this, _1),
          ros::VoidPtr(), &queue_);

    imuSub_ = nh_->subscribe(so);

	// Setup ROS Publishers
	// enc_fr_publisher_ = nh_->advertise<terrasentia_sensors::TerraEncoder>(fr_topic, 1);


    // start custom queue for imu subscriber
    this->callback_queue_thread_ = boost::thread(boost::bind(&CameraGimbalPlugin::QueueThread, this));

      // listen to the update event (broadcast every simulation iteration)
     this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&CameraGimbalPlugin::ExplicitUpdate, this));

	// Declare initial update time
	this->prevUpdateTime = this->model->GetWorld()->SimTime();
}

/////////////////////////////////////////////////
void CameraGimbalPlugin::ExplicitUpdate(){
	common::Time currTime = this->model->GetWorld()->SimTime();
	double dt = (currTime - this->prevUpdateTime).Double();

	// Update Ground Truth for Joint Angle regardless of PID step
	bool stopJoint = this->isJointLimitReached();
	double jointVel = this->jointHandle->GetVelocity(0);

	// Update PID Controller (if enough time has passed)
	if(dt > this->update_period_){
		// TODO: Somehow dynamically update target angle
		double tmpTargetAngle;
		if(nh_->getParam(this->target_angle_param, tmpTargetAngle)){
			this->setTargetAngle(tmpTargetAngle);
			// if(this->verbose) std::cout << "\r\n\r\n	RECEIVED NEW TARGET ANGLE: " << tmpTargetAngle << std::endl;
		}

		// Get most current IMU angles
		this->updateImuAngles();

		// Determine & Update the IMU axis to be used for PID feedback
		double pidInput = this->getFeedbackAngle();

		// Calculate new PID control commands
		double newCmd = this->pidStep(pidInput,dt);

		// Perform any final modifications to the calculated PID controls before being used
		double cmdGain = this->flip_control_direction_ ? -this->control_gain_ : this->control_gain_;
		double modifiedCmd = newCmd * cmdGain;

		// Update last known variables of this update step for next iterations
		this->prevUpdateTime += common::Time(this->update_period_);
		this->prev_input = pidInput;
		this->prev_output = modifiedCmd;

		// TODO: Figure out how to properly set control velocity during the time where no updates are being performed
		// this->jointHandle->SetVelocity(0, this->prev_output);
		this->jointHandle->SetParam("fmax", 0, 100.0);
		this->jointHandle->SetParam("vel", 0, this->prev_output);
		// std::cout << "\r\n\r\n	Control Joint Velocity: " << this->prev_output << std::endl;
		// std::cout << "[ ----- IMU ----- ] Angles (deg): " << this->imuRoll*M_RAD2DEG << ", " << this->imuPitch*M_RAD2DEG << ", " << this->imuYaw*M_RAD2DEG << std::endl;
	}


	// gzdbg << "[Joint Angle (deg), IMU RPY Angles (deg), dAngles]: " << degrees << ",      [" << this->imuRoll*M_RAD2DEG << ", " << this->imuPitch*M_RAD2DEG << ", " << this->imuYaw*M_RAD2DEG << "]" << ",      [" << ang_diff1 << ", " << ang_diff2 << ", " << ang_diff3 << "]" << std::endl;

	// if(stopJoint){
	// 	// Stop all joint movement
	// 	this->jointHandle->SetVelocity(0, 0);
	// 	// this->jointHandle->SetForce(0, force);
	// }

	// double ang_diff1 = (this->trueJointAngle - this->imuRoll) * M_RAD2DEG;
	// double ang_diff2 = (this->trueJointAngle - this->imuPitch) * M_RAD2DEG;
	// double ang_diff3 = (this->trueJointAngle - this->imuYaw) * M_RAD2DEG;
	// double angDiff = (this->trueJointAngle - correctedAngle) * M_RAD2DEG;

     // double force = -this->springStiffness * (pos - this->springReference) - this->springDamping * vel;

	// if(this->verbose) gzdbg << "[Joint Angle (deg), IMU RPY Angles (deg), dAngles]: " << degrees << ",      [" << this->imuRoll*M_RAD2DEG << ", " << this->imuPitch*M_RAD2DEG << ", " << this->imuYaw*M_RAD2DEG << "]" << ",      [" << ang_diff1 << ", " << ang_diff2 << ", " << ang_diff3 << "]" << std::endl;
     // ROS_INFO_NAMED("TorsionalSpring", "Pos, Vel, Force: %f, %f, %f \r\n", pos, vel, force);

}

bool CameraGimbalPlugin::isJointLimitReached(bool enforce_limits){
	// Get the current angle of the joint
	double jointAng = this->jointHandle->Position(0);

	if(jointAng >= this->maxAngle_){ // Return true if the current joint angle is outside either of its limits
		// gzdbg << "\r\n	[isJointLimitReached] ----- Maximum Joint Angle (" << jointAngDeg << ") Limit Reached!\r\n" << std::endl;
		if(enforce_limits) this->jointHandle->SetPosition(0, this->maxAngle_);
		return true;
	}else if(jointAng <= this->minAngle_){
		// gzdbg << "\r\n	[isJointLimitReached] ----- Minimum Joint Angle (" << jointAngDeg << ") Limit Reached!\r\n" << std::endl;
		if(enforce_limits) this->jointHandle->SetPosition(0, this->minAngle_);
		return true;
	} else{
		return false;
	}
}


void CameraGimbalPlugin::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg){
	double tmpRoll, tmpPitch, tmpYaw, tmpJointAngle, choiceAngle;

	boost::mutex::scoped_lock scoped_lock(lock);

	// Get the True Angle of the Joint
	tmpJointAngle = this->jointHandle->Position(0);

	// Update and convert IMU angular information
	tf::Quaternion q( imu_msg->orientation.x,
				   imu_msg->orientation.y,
				   imu_msg->orientation.z,
				   imu_msg->orientation.w
				);
	tf::Matrix3x3 m(q);
	m.getRPY(tmpRoll, tmpPitch, tmpYaw);

	// Store within class
	this->lockedJointTruth = tmpJointAngle;
	this->lockedRoll = tmpRoll;
	this->lockedPitch = tmpPitch;
	this->lockedYaw = tmpYaw;

	/** Initial Code used to debug angle offsets and whatnot

	double degrees = this->jointHandle->GetAngle(0).Degree();
	double ang_diff1 = (this->trueJointAngle - this->lockedRoll) * M_RAD2DEG;
	double ang_diff2 = (this->trueJointAngle - this->lockedPitch) * M_RAD2DEG;
	double ang_diff3 = (this->trueJointAngle - this->lockedYaw) * M_RAD2DEG;

	if(this->useImuAxisFlag == 1){ 		// Use Roll axis
		choiceAngle = this->lockedRoll;
	}else if(this->useImuAxisFlag == 2){	// Use Pitch axis
		choiceAngle = this->lockedPitch;
	}else{				// Use Yaw axis by default in case of user-input error
		choiceAngle = this->lockedYaw;
	}

	double correctedAngle = choiceAngle + this->imuOffset;
	double angDiff = (this->trueJointAngle - correctedAngle) * M_RAD2DEG;

	*/

	// if(this->verbose) gzdbg << "[Joint Angle (deg), IMU RPY Angles (deg), dAngles]: " << degrees << ",      [" << this->imuRoll*M_RAD2DEG << ", " << this->imuPitch*M_RAD2DEG << ", " << this->imuYaw*M_RAD2DEG << "]" << ",      [" << ang_diff1 << ", " << ang_diff2 << ", " << ang_diff3 << "]" << std::endl;
	// if(this->verbose) gzdbg << "[Joint Angle (deg), IMU Angle (deg), dAngles]: " << degrees << ", " << correctedAngle*M_RAD2DEG << ", " << angDiff << std::endl;
	// if(this->verbose) std::cout << "\r\n[ImuCallback] --- [IMU RPY Angles (deg)]: " << tmpRoll*M_RAD2DEG << ", " << tmpPitch*M_RAD2DEG << ", " << tmpYaw*M_RAD2DEG << std::endl;
}

double CameraGimbalPlugin::pidStep(double _sensor_feedback, double _dt){

	// Calculate error
	double error = this->_target_angle - _sensor_feedback*M_RAD2DEG;

	// Proportional term
	double Pout = this->kP * error;

	// Integral term
	this->_integral += (this->kI * error * _dt);
	// Saturate Integral error to help prevent "Integral Windup"
	if(this->_integral > this->maxIntegralErr_) this->_integral = this->maxIntegralErr_;
	else if(this->_integral < this->minIntegralErr_) this->_integral = this->minIntegralErr_;

	// Derivative term
	double dInput = (_sensor_feedback*M_RAD2DEG - this->prev_input*M_RAD2DEG) / _dt;
	double Dout = this->kD * dInput;

	// Calculate PID controller output command
	double output = Pout + this->_integral - Dout;

	// Restrict to max/min
	if(output > this->maxCmd_) output = this->maxCmd_;
	else if(output < this->minCmd_) output = this->minCmd_;

	// Prevent Motor from executing any commands in the direction opposite of the desired direction of motion
	// if(_target > 0) output = fabs(output);
	// else if(_target < 0) output = -1 * fabs(output);

	gzdbg << "[ ----- PID ----- ] Target, Input, Error, Integral, Output: " << this->_target_angle << ", " << _sensor_feedback*M_RAD2DEG << ", " << error << ", " << this->_integral << ", " << output << std::endl;
	return output;
}

void CameraGimbalPlugin::updateImuAngles(){
	double tmpRoll, tmpPitch, tmpYaw, tmpTruth;
	boost::mutex::scoped_lock scoped_lock(lock);

	tmpRoll = this->lockedRoll;
	tmpPitch = this->lockedPitch;
	tmpYaw = this->lockedYaw;
     tmpTruth = this->lockedJointTruth;

	this->imuRoll = tmpRoll;
	this->imuPitch = tmpPitch;
	this->imuYaw = tmpYaw;
     this->trueAngle = tmpTruth;
}

void CameraGimbalPlugin::setTargetAngle(double angle){
	boost::mutex::scoped_lock scoped_lock(lock);
	this->_target_angle = angle;
}

double CameraGimbalPlugin::getFeedbackAngle(bool unwrap){
	double choiceAngle;

	// Decide which IMU axis to use
	if(this->useImuAxisFlag == 1){ 		// Use Roll axis
		choiceAngle = this->imuRoll;
	}else if(this->useImuAxisFlag == 2){	// Use Pitch axis
		choiceAngle = this->imuPitch;
	}else if(this->useImuAxisFlag == 3){	// Use Yaw axis
		choiceAngle = this->imuYaw;
	} else{                                // Use ground truth angle otherwise
          choiceAngle = this->trueAngle;
          return choiceAngle;
     }

     /** If we aren't using the ground truth joint angle
     *         then we will need to do some pre-processing of IMU angles
     */


     double maxWrap = 180;
     double minWrap = -180;

	double tmpAngle = this->use_offset_ ? choiceAngle + this->imuOffset : choiceAngle;

     double tmpDeg = tmpAngle*M_RAD2DEG;
     double in1 = tmpDeg - minWrap; double in2 = maxWrap - minWrap;

     double preUnwrapped = fmod(in2 + fmod(in1, in2), in2);
     double unwrappedAngle = (minWrap + preUnwrapped);

     // unwrappedAngle = this->use_offset_ ? unwrappedAngle + this->imuOffset : unwrappedAngle;

     double outputAngle = unwrap ? unwrappedAngle*M_DEG2RAD : tmpAngle;

     std::cout << "[ ----- Angles ----- ] Raw Angle, Offset Angle, UnWrapped Angle: " << choiceAngle*M_RAD2DEG << ", " << tmpAngle*M_RAD2DEG << ", " << unwrappedAngle << std::endl;

	return outputAngle;

}

void CameraGimbalPlugin::QueueThread(){
	static const double timeout = 0.01;
	while(alive_ && nh_->ok()){
		queue_.callAvailable(ros::WallDuration(timeout));
	}
}

void CameraGimbalPlugin::FiniChild() {
	alive_ = false;
	queue_.clear();
	queue_.disable();
	nh_->shutdown();
	callback_queue_thread_.join();
}
