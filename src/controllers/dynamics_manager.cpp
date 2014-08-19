#include <iostream>
#include <algorithm>
#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8.h>

#include "control_group.h"

#include <dynamic_models/ControlGroup.h>
#include <dynamic_models/AddGroup.h>
#include <dynamic_models/DeleteGroup.h>
#include <dynamic_models/ListGroups.h>
#include <dynamic_models/OpenCloseDoors.h>
#include <dynamic_models/OpenCloseElevDoors.h>
#include <dynamic_models/SetElevProps.h>
#include <dynamic_models/SetVelDoors.h>
#include <dynamic_models/TargetFloorElev.h>

#define TYPE_DOOR_STR "door"
#define TYPE_ELEVATOR_STR "elevator"

#define DEFAULT_SLIDE_SPEED 1 // in m/s
#define DEFAULT_FLIP_SPEED 1.57 // in rad/s

#define STATE_OPEN true
#define STATE_CLOSE false

#define ELEV_DOOR_STATE_OPEN 1
#define ELEV_DOOR_STATE_CLOSE 0
#define ELEV_DOOR_STATE_FREE 2

#define INDEX_NOT_FOUND -1

/*

Limitations:
	Topic messages are dropped sometimes while making the first service call
*/

class DynamicsController
{
	private:

		ros::NodeHandle rosNode;
		ros::ServiceServer add_group_server, delete_group_server, list_groups_server;
		ros::ServiceServer open_close_doors_server, set_vel_doors_server, target_floor_elev_server, set_elev_props_server, open_close_elev_doors_server;
		
		ros::Publisher door_cmd_vel_pub, door_active_pub;
		ros::Publisher elev_target_pub, elev_active_pub, elev_param_pub, elev_door_pub;

		std::vector<ControlGroup> groups;

	public:

		DynamicsController(ros::NodeHandle &nh)
		{
			rosNode = nh;
			nh = ros::NodeHandle("");

			setupControlTopics();
			setupManagerServices();
		}

		void setupManagerServices()
		{
			add_group_server = rosNode.advertiseService("model_dynamics_manager/add_control_group", &DynamicsController::add_control_group_cb, this);
			delete_group_server = rosNode.advertiseService("model_dynamics_manager/delete_control_group", &DynamicsController::delete_control_group_cb, this);
			list_groups_server = rosNode.advertiseService("model_dynamics_manager/list_groups", &DynamicsController::list_groups_cb, this);

			open_close_doors_server = rosNode.advertiseService("model_dynamics_manager/doors/open_close", &DynamicsController::open_close_doors_cb, this);
			set_vel_doors_server = rosNode.advertiseService("model_dynamics_manager/doors/set_vel", &DynamicsController::set_vel_doors_cb, this);

			target_floor_elev_server = rosNode.advertiseService("model_dynamics_manager/elevators/target_floor", &DynamicsController::target_floor_elev_cb, this);
			set_elev_props_server = rosNode.advertiseService("model_dynamics_manager/elevators/set_props", &DynamicsController::set_elev_props_cb, this);
			open_close_elev_doors_server = rosNode.advertiseService("model_dynamics_manager/elevators/open_close_elev", &DynamicsController::open_close_elev_cb, this);		
		}

		bool open_close_doors_cb(dynamic_models::OpenCloseDoors::Request &req, dynamic_models::OpenCloseDoors::Response &res)
		{
			if (!activateDoors(req.group_name)) {
				return false;
			}

			geometry_msgs::Twist cmd_vel;

			if (req.state == STATE_OPEN) {
				cmd_vel.linear.x = -DEFAULT_SLIDE_SPEED;
				cmd_vel.linear.y = -DEFAULT_SLIDE_SPEED;
				cmd_vel.angular.z = -DEFAULT_FLIP_SPEED;
			} else {
				cmd_vel.linear.x = DEFAULT_SLIDE_SPEED;
				cmd_vel.linear.y = DEFAULT_SLIDE_SPEED;
				cmd_vel.angular.z = DEFAULT_FLIP_SPEED;
			}

			door_cmd_vel_pub.publish(cmd_vel);

			return true;
		}

		bool set_vel_doors_cb(dynamic_models::SetVelDoors::Request &req, dynamic_models::SetVelDoors::Response &res)
		{
			if (!activateDoors(req.group_name)) {
				return false;
			}

			geometry_msgs::Twist cmd_vel;

			cmd_vel.linear.x = req.lin_x;
			cmd_vel.linear.y = req.lin_y;
			cmd_vel.angular.z = req.ang_z;

			door_cmd_vel_pub.publish(cmd_vel);

			return true;
		}

		bool target_floor_elev_cb(dynamic_models::TargetFloorElev::Request &req, dynamic_models::TargetFloorElev::Response &res)
		{
			if (!activateElevators(req.group_name)) {
				return false;
			}

			std_msgs::UInt8 elev_door_state;
			elev_door_state.data = ELEV_DOOR_STATE_FREE;

			std_msgs::Int32 target_floor;
			target_floor.data = req.target_floor;

			elev_door_pub.publish(elev_door_state);
			elev_target_pub.publish(target_floor);

			return true;
		}

		bool set_elev_props_cb(dynamic_models::SetElevProps::Request &req, dynamic_models::SetElevProps::Response &res)
		{
			if (!activateElevators(req.group_name)) {
				return false;
			}

			std_msgs::Float32MultiArray elev_params;
			elev_params.data.push_back(req.velocity);
			elev_params.data.push_back(req.force);

			elev_param_pub.publish(elev_params);

			return true;
		}

		bool open_close_elev_cb(dynamic_models::OpenCloseElevDoors::Request &req, dynamic_models::OpenCloseElevDoors::Response &res)
		{
			if (!activateElevators(req.group_name)) {
				return false;
			}

			std_msgs::UInt8 elev_door_state;

			if (req.state == STATE_OPEN) {
				elev_door_state.data = ELEV_DOOR_STATE_OPEN;
			} else {
				elev_door_state.data = ELEV_DOOR_STATE_CLOSE;
			}

			elev_door_pub.publish(elev_door_state);

			return true;
		}

		bool activateDoors(std::string group_name)
		{
			int groupIndex = getGroupIndex(group_name);

			if (groupIndex == INDEX_NOT_FOUND) {
				ROS_ERROR("Door Service Failed: The specified group does not exist");
				return false;
			}

			ControlGroup currGroup = groups.at(groupIndex);

			if (currGroup.getType() != DOOR) {
				ROS_ERROR("Door Service Failed: This group type doesn't support this call");
				return false;
			}

			// Publish the IDs of the active doors in the group
			std_msgs::UInt32MultiArray active_doors = uintVectorToStdMsgs(currGroup.getActiveUnits());
			door_active_pub.publish(active_doors);

			return true;
		}

		bool activateElevators(std::string group_name)
		{
			int groupIndex = getGroupIndex(group_name);

			if (groupIndex == INDEX_NOT_FOUND) {
				ROS_ERROR("Elevator Service Failed: The specified group does not exist");
				return false;
			}

			ControlGroup currGroup = groups.at(groupIndex);

			if (currGroup.getType() != ELEVATOR) {
				ROS_ERROR("Elevato Service Failed: This group type doesn't support this call");
				return false;
			}

			std_msgs::UInt32MultiArray active_elevs = uintVectorToStdMsgs(currGroup.getActiveUnits());
			elev_active_pub.publish(active_elevs);

			return true;
		}

		void setupControlTopics()
		{
			door_cmd_vel_pub = rosNode.advertise<geometry_msgs::Twist>("/door_controller/command", 10);
			door_active_pub = rosNode.advertise<std_msgs::UInt32MultiArray>("/door_controller/active", 10);

		    elev_target_pub = rosNode.advertise<std_msgs::Int32>("/elevator_controller/target_floor", 10);
		    elev_active_pub = rosNode.advertise<std_msgs::UInt32MultiArray>("elevator_controller/active", 10);
		    elev_param_pub = rosNode.advertise<std_msgs::Float32MultiArray>("elevator_controller/param", 10);
		    elev_door_pub = rosNode.advertise<std_msgs::UInt8>("/elevator_controller/door", 10);
		}

		std_msgs::UInt32MultiArray uintVectorToStdMsgs(std::vector<uint32_t> active_units)
		{
			std_msgs::UInt32MultiArray active_list;

			for (int i=0; i<active_units.size(); i++)
			{	
				active_list.data.push_back(active_units.at(i));
			}

			return active_list;
		}

		bool add_control_group_cb(dynamic_models::AddGroup::Request &req, dynamic_models::AddGroup::Response &res)
		{	
			GroupType type = parseGroupType(req.group.type);

			if (type == INVALID) {
				ROS_ERROR("Add Group Service Failed: Invalid group type");
				return false;
			}

			// Check if there is an exiting group with the same name, which might cause conflicts:			
			int groupIndex = getGroupIndex(req.group.group_name);
			if (groupIndex != INDEX_NOT_FOUND) {
				ROS_ERROR("Add Group Service Failed: The specified group name already exists");
				return false;				
			}

			ControlGroup group(req.group.group_name, type, req.group.active_units);
			groups.push_back(group);

			return true;
		}

		bool delete_control_group_cb(dynamic_models::DeleteGroup::Request &req, dynamic_models::DeleteGroup::Response &res)
		{
			int groupIndex = getGroupIndex(req.group_name);

			if (groupIndex == INDEX_NOT_FOUND) {
				ROS_WARN("Delete Group Service: The specified group does not exist");
				return false;
			}

			groups.erase(groups.begin() + groupIndex);

			return true;
		}

		bool list_groups_cb(dynamic_models::ListGroups::Request &req, dynamic_models::ListGroups::Response &res)
		{
			for (int i=0; i<groups.size(); i++) {
				dynamic_models::ControlGroup item;

				item.group_name = groups.at(i).getGroupName();
				item.type = groups.at(i).getType();
				item.active_units = groups.at(i).getActiveUnits();

				res.groups.push_back(item);
			}
			
			return true;
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

		int getGroupIndex(std::string group_name)
		{
			int groupIndex = INDEX_NOT_FOUND;

			for (int i=0; i<groups.size(); i++) {
				if (groups.at(i).getGroupName().compare(group_name) == 0) {
					groupIndex = i;
					return groupIndex;
				}
			}

			return groupIndex;
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
