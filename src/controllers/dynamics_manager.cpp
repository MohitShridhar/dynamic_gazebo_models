#include <iostream>
#include <ros/ros.h>

#include "control_group.cpp"

#include <dynamic_models/AddGroup.h>
#include <dynamic_models/DeleteGroup.h>

#define TYPE_DOOR_STR "door"
#define TYPE_ELEVATOR_STR "elevator"

class DynamicsController
{
	private:

		ros::NodeHandle rosNode;
		ros::ServiceServer add_group_server, delete_group_server;

		std::vector<ControlGroup> groups;		

	public:

		DynamicsController(ros::NodeHandle &nh)
		{
			rosNode = nh;
			nh = ros::NodeHandle("");

			setupManagerServices();

		}

		void setupManagerServices()
		{
			add_group_server = rosNode.advertiseService("model_dynamics_manager/add_control_group", &DynamicsController::add_control_group_cb, this);
			delete_group_server = rosNode.advertiseService("model_dynamics_manager/delete_control_group", &DynamicsController::delete_control_group_cb, this);
		}

		bool add_control_group_cb(dynamic_models::AddGroup::Request &req, dynamic_models::AddGroup::Response &res)
		{	
			GroupType type = parseGroupType(req.type);

			if (type == INVALID) {
				ROS_ERROR("Add Group Service Failed: Invalid group type.");
				return false;
			}

			ControlGroup group(req.group_name, type, req.active_units);
		}

		bool delete_control_group_cb(dynamic_models::DeleteGroup::Request &req, dynamic_models::DeleteGroup::Response &res)
		{

		}

		GroupType parseGroupType(std::string type_str)
		{	
			if (type_str.compare(TYPE_DOOR_STR) == 0) {
				return DOOR;
			} else if (type_str.compare(TYPE_ELEVATOR_STR) == 0) {
				return ELEVATOR;
			} else {
				return INVALID; // invalid type in request
			}
		}

		void start()
		{
			while (rosNode.ok()) {
				ros::spinOnce();
			}
		}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "dynamic_model_contrroler");
	ros::NodeHandle rosNode;

	DynamicsController controller(rosNode);
	controller.start();
}
