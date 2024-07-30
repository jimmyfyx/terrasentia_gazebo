#ifndef __TORSIONAL_SPRING_PLUGIN_HH__
#define __TORSIONAL_SPRING_PLUGIN_HH__

#include <string>

#include "gazebo/common/common.hh"
#include "gazebo/common/Events.hh"

#include "gazebo/gazebo.hh"

#include "gazebo/physics/physics.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/Joint.hh"

namespace gazebo{
	class TorsionalSpringPlugin : public ModelPlugin{
		private:
			void ExplicitUpdate();
			event::ConnectionPtr updateConnection;
			common::Time prevUpdateTime;
			physics::ModelPtr model;

			physics::JointPtr jointHandle;
			std::string jointName;
			double springStiffness;
			double springDamping;
			double springReference;
			bool verbose;
		public:

			TorsionalSpringPlugin();
			virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
			virtual void Init();
	};
}
#endif
