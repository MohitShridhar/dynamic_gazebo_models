#include <iostream>
#include <ros/ros.h>

class DynamicsController
{
	private:

		ros::NodeHandle rosNode;

	public:

		DynamicsController(ros::NodeHandle &nh)
		{
			rosNode = nh;
		}

		void start()
		{
			ros::spinOnce();
		}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "dynamic_model_contrroler");
	ros::NodeHandle rosNode;

	DynamicsController controller(rosNode);
	controller.start();
}
