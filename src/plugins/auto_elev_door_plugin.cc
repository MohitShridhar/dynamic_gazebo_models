#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

#include <ros/ros.h>
#include <std_msgs/UInt32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Twist.h>

#define DEFAULT_SLIDE_DISTANCE 0.711305
#define DEFAULT_SLIDE_SPEED 1 // in m/s

#define HEIGHT_LEVEL_TOLERANCE 0.01

namespace gazebo
{
	class AutoElevDoorPlugin : public ModelPlugin
	{
		private:
			ros::NodeHandle *rosNode;

		public: 

			AutoElevDoorPlugin()
			{
		      std::string name = "auto_elevator_door_plugin";
		      int argc = 0;
		      ros::init(argc, NULL, name);
			}

			~AutoElevDoorPlugin()
			{
				delete this->rosNode;
			}

			void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
			{

			}

	};

	GZ_REGISTER_MODEL_PLUGIN(AutoElevDoorPlugin);
}