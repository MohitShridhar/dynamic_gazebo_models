#include <iostream>

#include <ros/ros.h>

class KeyboardOp
{
	private:
		ros::NodeHandle rosNode;

	public:
		KeyboardOp(ros::NodeHandle &nh)
		{
			rosNode = nh;
		}

		void start()
		{

		}
};


int main(int argc, char** argv)
{

  ros::init(argc, argv, "keyboard_op_model_dynamics_control");
  ros::NodeHandle nh;

  KeyboardOp controller(nh);
  controller.start();
}