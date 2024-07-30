#include <ros/ros.h>
#include "gazebo/physics/physics.hh"
#include "TorsionalSpringPlugin.hh"
using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(TorsionalSpringPlugin)

TorsionalSpringPlugin::TorsionalSpringPlugin(){}

void TorsionalSpringPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){

	this->model = _model;

	if(!_sdf->HasElement("joint_handle")){
		ROS_INFO_NAMED("libTorsionalSpringPlugin", "Plugin missing <joint_handle>, defaults to /base_link");
		this->jointName = "/base_link";
	} else this->jointName = _sdf->Get<std::string>("joint_handle");

	if(!_sdf->HasElement("spring_stiffness")){
		ROS_INFO_NAMED("libTorsionalSpringPlugin", "Plugin missing <spring_stiffness>, defaults to 100.0");
		this->springStiffness = 100.0;
	} else this->springStiffness = _sdf->Get<double>("spring_stiffness");

	if(!_sdf->HasElement("spring_damping")){
		ROS_INFO_NAMED("libTorsionalSpringPlugin", "Plugin missing <spring_damping>, defaults to 1.0");
		this->springDamping = 1.0;
	} else this->springDamping = _sdf->Get<double>("spring_damping");

	if(!_sdf->HasElement("spring_reference")){
		ROS_INFO_NAMED("libTorsionalSpringPlugin", "Plugin missing <spring_reference>, defaults to 1.0");
		this->springReference = 1.0;
	} else this->springReference = _sdf->Get<double>("spring_reference");

	if(!_sdf->HasElement("verbose")){
		ROS_INFO_NAMED("libTorsionalSpringPlugin", "Plugin missing <verbose>, defaults to false");
		this->verbose = false;
	} else this->verbose = _sdf->Get<bool>("verbose");

}

void TorsionalSpringPlugin::Init(){
	this->jointHandle = this->model->GetJoint(this->jointName);
	this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&TorsionalSpringPlugin::ExplicitUpdate, this));
}

void TorsionalSpringPlugin::ExplicitUpdate(){
	common::Time currTime = this->model->GetWorld()->SimTime();
	common::Time stepTime = currTime - this->prevUpdateTime;
	this->prevUpdateTime = currTime;

	double pos = this->jointHandle->Position(0);
	double vel = this->jointHandle->GetVelocity(0);
	double force = -this->springStiffness * (pos - this->springReference) - this->springDamping * vel;

	if(this->verbose) gzdbg << "[Joint] ----- Pos, Vel, Force: " << this->jointName << ",      " << pos << ",      " << vel << ",      " << force << std::endl;
	// ROS_INFO_NAMED("TorsionalSpring", "Pos, Vel, Force: %f, %f, %f \r\n", pos, vel, force);

	this->jointHandle->SetForce(0, force);
}
