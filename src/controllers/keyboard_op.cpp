#include <iostream>
#include <boost/algorithm/string.hpp>

#include <ros/ros.h>

#include <dynamic_models/ControlGroup.h>
#include <dynamic_models/AddGroup.h>
#include <dynamic_models/DeleteGroup.h>
#include <dynamic_models/ListGroups.h>
#include <dynamic_models/OpenCloseDoors.h>
#include <dynamic_models/OpenCloseElevDoors.h>
#include <dynamic_models/SetElevProps.h>
#include <dynamic_models/SetVelDoors.h>
#include <dynamic_models/TargetFloorElev.h>

#define CONTROL_GROUP_NAME "keyboard_op_control_group"

#define DEFAULT_ELEV_SPEED 1.5
#define DEFAULT_ELEV_FORCE 100

#define DEFAULT_SLIDE_SPEED 1.0
#define DEFAULT_FLIP_SPEED 1.57

enum ControlType {DOOR, ELEVATOR};

class KeyboardOp
{
	private:
		ros::NodeHandle rosNode;
		ros::ServiceClient add_group_client, delete_group_client, list_groups_client;
		ros::ServiceClient open_close_doors_client, set_vel_doors_client, target_floor_elev_client, set_elev_props_client, open_close_elev_doors_client;

		ControlType type;
		std::string groupName;
		bool isGroupInitialized;

		dynamic_models::OpenCloseDoors openDoorsCall;
		dynamic_models::OpenCloseDoors closeDoorsCall;
		dynamic_models::SetVelDoors setVelDoorsCall;
		dynamic_models::TargetFloorElev targetFloorCall;
		dynamic_models::OpenCloseElevDoors openElevDoorsCall;
		dynamic_models::OpenCloseElevDoors closeElevDoorsCall;
		dynamic_models::SetElevProps setElevSpeedCall;
		dynamic_models::SetElevProps setElevForceCall;

	public:
		KeyboardOp(ros::NodeHandle &nh)
		{
			rosNode = nh;
			rosNode = ros::NodeHandle("");

			setupClientServices();
			initVars();
		}

		void initVars()
		{
			type = DOOR;
		}

		void setupClientServices()
		{
			add_group_client = rosNode.serviceClient<dynamic_models::AddGroup>("model_dynamics_manager/add_control_group");
			delete_group_client = rosNode.serviceClient<dynamic_models::DeleteGroup>("model_dynamics_manager/delete_control_group");
			list_groups_client = rosNode.serviceClient<dynamic_models::ListGroups>("model_dynamics_manager/list_groups");

			open_close_doors_client = rosNode.serviceClient<dynamic_models::OpenCloseDoors>("model_dynamics_manager/doors/open_close");
			set_vel_doors_client = rosNode.serviceClient<dynamic_models::SetVelDoors>("model_dynamics_manager/doors/set_vel");

			target_floor_elev_client = rosNode.serviceClient<dynamic_models::TargetFloorElev>("model_dynamics_manager/elevators/target_floor");
			set_elev_props_client = rosNode.serviceClient<dynamic_models::SetElevProps>("model_dynamics_manager/elevators/set_props");
			open_close_elev_doors_client = rosNode.serviceClient<dynamic_models::OpenCloseElevDoors>("model_dynamics_manager/elevators/open_close_elev");	
		}

		bool setControlType(char input[])
		{
			std::string inputStr(input);

			if (boost::iequals(inputStr, "door")) {
				std::cout << "Control type set to 'door'" << std::endl;
				type = DOOR;
				setActiveUnits();
				return true;
			} else if (boost::iequals(inputStr, "elevator")){
				std::cout << "Control type set to 'elevator'" << std::endl;
				type = ELEVATOR;
				setActiveUnits();
				return true;
			}

			return false;
		}

		void setActiveUnits()
		{
			char input[30];
			std::cout << "Enter the reference numbers of the units you want to control. Eg: 1, 3, 4 for units one, three & four" << std::endl;
			
			readLineInput(input);
			std::vector<uint32_t> activeList = parseActiveList(input);
			
			// Delete previous group if already initialized. Note: IGNORE the warning produced during initialization about delete service failing
			dynamic_models::DeleteGroup deleteSrv;
			deleteSrv.request.group_name = CONTROL_GROUP_NAME;
			delete_group_client.call(deleteSrv);

			// Add new group with the desired units
			dynamic_models::AddGroup addSrv;
			addSrv.request.group.group_name = CONTROL_GROUP_NAME;
			addSrv.request.group.type = type == DOOR ? "door" : "elevator";
			addSrv.request.group.active_units = activeList;
			add_group_client.call(addSrv);

			isGroupInitialized = true;
		}

	    std::vector<uint32_t> parseActiveList(char input[])
	    {
	      std::string active_list_str(input);
	      std::vector<uint32_t> active_list;

	      // parse csv-style input (also remove whitespace):
	      std::string::iterator end_pos = std::remove(active_list_str.begin(), active_list_str.end(), ' ');
	      active_list_str.erase(end_pos, active_list_str.end());

	      std::istringstream ss(active_list_str);
	      std::string token;

	      while (std::getline(ss, token, ',')) {
	      	try {
	        	active_list.push_back(atoi(token.c_str()));
	      	} catch (...) {
	      		std::cout << "Invalid active list. Exiting.." << std::endl;
	      		std::exit(EXIT_SUCCESS);
	      	}
	      }

	      return active_list;
	    }

		void initialize()
		{
			char input[30];
			isGroupInitialized = false;

			std::cout << "Enter the |type| of models you want to control: 'door' or 'elevator'" << std::endl;

			readLineInput(input);
			while (!setControlType(input)) {
				std::cout << "Invalid type. Options: 'door' or 'elevator'" << std::endl;
				readLineInput(input);
			}

			setupCallTemplates();
		}

		void setupCallTemplates()
		{
			// DOOR based services:
			openDoorsCall.request.group_name = CONTROL_GROUP_NAME;
			openDoorsCall.request.state = true;

			closeDoorsCall.request.group_name = CONTROL_GROUP_NAME;
			closeDoorsCall.request.state = false;

			setVelDoorsCall.request.group_name = CONTROL_GROUP_NAME;
			setVelDoorsCall.request.lin_x = DEFAULT_SLIDE_SPEED;
			setVelDoorsCall.request.lin_y = DEFAULT_SLIDE_SPEED;
			setVelDoorsCall.request.ang_z = DEFAULT_FLIP_SPEED;

			// ELEVATOR based services:
			targetFloorCall.request.group_name = CONTROL_GROUP_NAME;
			targetFloorCall.request.target_floor = 0;

			openElevDoorsCall.request.group_name = CONTROL_GROUP_NAME;
			openElevDoorsCall.request.state = true;

			closeElevDoorsCall.request.group_name = CONTROL_GROUP_NAME;
			closeElevDoorsCall.request.state = false;

			setElevSpeedCall.request.group_name = CONTROL_GROUP_NAME;
			setElevSpeedCall.request.velocity = DEFAULT_ELEV_SPEED;
			setElevSpeedCall.request.force = DEFAULT_ELEV_FORCE;

			setElevForceCall.request.group_name = CONTROL_GROUP_NAME;
			setElevForceCall.request.velocity = DEFAULT_ELEV_SPEED;
			setElevForceCall.request.force = DEFAULT_ELEV_FORCE;
		}

		void readLineInput(char input[30])
		{
			std::cin.getline(input, 30);

			std::string inputStr(input);
			if (boost::iequals(inputStr, "q")) {
				rosNode.shutdown();				
				std::exit(EXIT_SUCCESS);
			}
		}

		void start()
		{
			initialize();
			char input[30];

			while (rosNode.ok())
			{
				readLineInput(input);

				// check if the type was toggled (between 'door' & 'elevator')
				if (setControlType(input)) {
					continue;
				}

				callServices(input);
			}		

		}

		void callServices(char input[])
		{
			ROS_ASSERT(type == DOOR || type == ELEVATOR);

			if (type == DOOR) {
				executeDoorServices(input);
			} else if (type == ELEVATOR) {
				executeElevatorServices(input);
			}
		}

		void executeElevatorServices(char input[])
		{

		}

		void executeDoorServices(char input[])
		{
			std::string inputStr(input);

			switch(input[0]) {

				case 'o':
					open_close_doors_client.call(openDoorsCall);
					break;
				case 'c':
					open_close_doors_client.call(closeDoorsCall);
					break;
				case 'l':
					setVelDoorsCall.request.lin_x = setVelDoorsCall.request.lin_y = parseFloat(inputStr.substr(1));
					set_vel_doors_client.call(setVelDoorsCall);
					break;
				case 'a':
					setVelDoorsCall.request.ang_z = parseFloat(inputStr.substr(1));
					set_vel_doors_client.call(setVelDoorsCall);
					break;
				default:
					std::cout << "Unknown command" << std::endl;			

			};
		}

		float parseFloat(std::string input)
		{
		    std::string::iterator end_pos = std::remove(input.begin(), input.end(), ' ');
		    input.erase(end_pos, input.end()); 

		    return atof(input.c_str());	
		}
};


int main(int argc, char** argv)
{

  ros::init(argc, argv, "keyboard_op_model_dynamics_control");
  ros::NodeHandle nh;

  KeyboardOp controller(nh);
  controller.start();
}