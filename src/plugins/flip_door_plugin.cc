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

#define CONTEXT_SPACE_X_RANGE 2.0 // in m
#define CONTEXT_SPACE_Y_RANGE 2.0
#define CONTEXT_SPACE_Z_RANGE 2.0


namespace gazebo
{ 

  class FlipDoor : public ModelPlugin
  {

  private:
    physics::ModelPtr model;
    physics::LinkPtr doorLink;
    math::Pose currPose, currFpvPose; 

    math::Vector3 cmd_vel;

    bool isActive;
    int activeDoors[100];
    
    int door_ref_num;
    std::string door_model_name, door_direction, model_domain_space;

    ros::NodeHandle* rosNode;
    transport::NodePtr gazeboNode;
    event::ConnectionPtr updateConnection;

    transport::SubscriberPtr subFpvPose, subGzRequest;
    ros::Subscriber sub, sub_active;

  public:
    FlipDoor()
    {
      std::string name = "flip_door_plugin_node";
      int argc = 0;
      ros::init(argc, NULL, name);

    }
    ~FlipDoor()
    {
      delete rosNode;
    }

    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      establishLinks(_parent);
      determineRotationType(_sdf);
      determineModelDomain(_sdf);
      initVars();
    }

    void OnUpdate()
    {
      ros::spinOnce();
      doorLink->SetAngularVel(cmd_vel);
    }

  private:
    void determineRotationType(sdf::ElementPtr _sdf)
    {
      if (!_sdf->HasElement("door_direction")) {
        ROS_ERROR("Door direction not specified in the plugin reference");
        std::exit(EXIT_FAILURE);
      }

      door_direction = _sdf->GetElement("door_direction")->Get<std::string>();

      if (door_direction.compare("clockwise") != 0 && door_direction.compare("counter_clockwise") != 0) {
        ROS_ERROR("Invalid door direction specified. Only two states possible: 'clockwise' & 'counter_clockwise'");
      }

    }

    void determineModelDomain(sdf::ElementPtr _sdf)
    {
      if (!_sdf->HasElement("model_domain_space")) {
        ROS_ERROR("Model Domain Space not specified in the plugin reference");
        std::exit(EXIT_FAILURE);
      }

      model_domain_space = _sdf->GetElement("model_domain_space")->Get<std::string>();

      ROS_INFO("Door '%s' initialized - Direction: %s, Domain Space: %s\n", door_model_name.c_str(), door_direction.c_str(), model_domain_space.c_str());
    }

    void initVars()
    {
      isActive = false;

      std::string door_ref_num_str = door_model_name; 
      replaceSubstring(door_ref_num_str, model_domain_space, "");
      door_ref_num = atoi(door_ref_num_str.c_str());
    }

    void establishLinks(physics::ModelPtr _parent)
    {
      model = _parent;
      doorLink = model->GetLink("door");
      door_model_name = model->GetName();

      rosNode = new ros::NodeHandle("/door_controller");

      sub = rosNode->subscribe<geometry_msgs::Twist>("/door_controller/command", 1000, &FlipDoor::cmd_ang_cb, this );
      sub_active = rosNode->subscribe<std_msgs::UInt32MultiArray>("/door_controller/active", 1000, &FlipDoor::active_doors_cb, this);

      updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&FlipDoor::OnUpdate, this));
    }

    void cmd_ang_cb(const geometry_msgs::Twist::ConstPtr& msg)
    {
      if (isActive) {
        setAngularVel(msg->angular.z);
        ROS_INFO("Door '%s' - Angular z: [%f]", door_model_name.c_str(), msg->angular.z);
      }
    }

    void setAngularVel(float angularVel)
    {
      if (door_direction.compare("clockwise") == 0) { 
        cmd_vel.z = angularVel;
      } else {
        cmd_vel.z = -angularVel; 
      }
    }

    void active_doors_cb(const std_msgs::UInt32MultiArray::ConstPtr& array) 
    {
      isActive = false;
      
      int i=0;

      for (std::vector<uint32_t>::const_iterator it = array->data.begin(); it != array->data.end(); ++it) {
        activeDoors[i] = *it;
        i++;

        if (*it == door_ref_num) {
          isActive = true;
        }
      }
    }

    std::string replaceSubstring(std::string &s, std::string toReplace, std::string replaceWith)
    {
      return(s.replace(s.find(toReplace), toReplace.length(), replaceWith));
    }

  };

  GZ_REGISTER_MODEL_PLUGIN(FlipDoor)
}