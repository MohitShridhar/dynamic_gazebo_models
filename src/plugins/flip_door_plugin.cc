#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

#include <boost/shared_ptr.hpp>
#include <gazebo/transport/transport.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/transport/TransportIface.hh>

#include <ros/ros.h>
#include <std_msgs/UInt32MultiArray.h>
#include <geometry_msgs/Twist.h>

#define PI_HALF 1.57

//ps3 controller service requests:
#define REQUEST_UP_BTN "up_btn"
#define REQUEST_DOWN_BTN "down_btn"
#define REQUEST_RIGHT_BTN "right_btn"
#define REQUEST_LEFT_BTN "left_btn"

#define CONTEXT_SPACE_X_RANGE 2.0 // in m
#define CONTEXT_SPACE_Y_RANGE 2.0
#define CONTEXT_SPACE_Z_RANGE 2.0


namespace gazebo
{ 

  typedef const boost::shared_ptr<const msgs::Pose> PosePtr;

  class FlipDoor : public ModelPlugin
  {

    private:
      ros::NodeHandle *node;

    public:

      FlipDoor()
      {
        std::string name = "ros_door_plugin_node";
        int argc = 0;
        ros::init(argc, NULL, name);

      }
      ~FlipDoor()
      {
        delete this->node;
      }

      void Load(physics::ModelPtr _parent, sdf::ElementPtr)
      {

      }

      void OnUpdate()
      {

      }

  };

  GZ_REGISTER_MODEL_PLUGIN(FlipDoor)
}