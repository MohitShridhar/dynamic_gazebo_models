#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <iostream>
#include <algorithm>

#include <boost/shared_ptr.hpp>
#include <gazebo/transport/transport.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/transport/TransportIface.hh>

#include <ros/ros.h>
#include <std_msgs/UInt32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

#define DEFAULT_OPEN_VEL -1.57
#define DEFAULT_CLOSE_VEL 1.57

#define TYPE_FLIP_OPEN "flip"
#define TYPE_SLIDE_OPEN "slide"

#define DIRECTION_FLIP_CLOCKWISE "clockwise"
#define DIRECTION_FLIP_COUNTER_CLOCKWISE "counter_clockwise"
#define DIRECTION_SLIDE_LEFT "left"
#define DIRECTION_SLIDE_RIGHT "right"


#define CONTEXT_SPACE_X_RANGE 2.0 // in m
#define CONTEXT_SPACE_Y_RANGE 2.0
#define CONTEXT_SPACE_Z_RANGE 2.0

// TODO: Customizable Rotation and Translation limits

namespace gazebo
{ 
  enum DoorType {FLIP, SLIDE};

  class DoorPlugin : public ModelPlugin
  {

  private:
    physics::ModelPtr model;
    physics::LinkPtr doorLink;
    math::Pose currPose, currFpvPose; 

    math::Vector3 cmd_vel;

    bool isActive;
    int activeDoors[100];
    DoorType type;
    
    int door_ref_num;
    std::string door_type, door_model_name, door_direction, model_domain_space;

    ros::NodeHandle* rosNode;
    transport::NodePtr gazeboNode;
    event::ConnectionPtr updateConnection;

    transport::SubscriberPtr subFpvPose, subGzRequest;
    ros::Subscriber sub, sub_active;

  public:
    DoorPlugin()
    {
      std::string name = "door_plugin_node";
      int argc = 0;
      ros::init(argc, NULL, name);

    }
    ~DoorPlugin()
    {
      delete rosNode;
    }

    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      establishLinks(_parent);
      determineDoorType(_sdf);
      determineDoorDirection(_sdf);
      determineModelDomain(_sdf);
      initVars();
    }

    void OnUpdate()
    {
      ros::spinOnce();
      updateLinkVel();
    }

  private:
    void determineDoorType(sdf::ElementPtr _sdf)
    {
      if (!_sdf->HasElement("door_type")) {
        ROS_WARN("Door Type not specified. Defaulting to 'flip'");
        door_type = "flip";
      } else {
        door_type = _sdf->GetElement("door_type")->Get<std::string>();
      }

      if (door_type.compare(TYPE_SLIDE_OPEN) == 0) {
        type = SLIDE;
      } else {
        type = FLIP;
      }
    }

    void determineDoorDirection(sdf::ElementPtr _sdf)
    {
      if (!_sdf->HasElement("door_direction")) {
        if (type == FLIP) {
          ROS_WARN("Door direction not specified in the plugin reference. Defaulting to 'clockwise'");
          door_direction = DIRECTION_FLIP_CLOCKWISE;
        } else if (type == SLIDE) {
          ROS_WARN("Door direction not specified in the plugin reference. Defaulting to 'left'");
          door_direction = DIRECTION_SLIDE_LEFT;
        } 
      } else {
        door_direction = _sdf->GetElement("door_direction")->Get<std::string>();
        checkDirectionValidity();
      }
    }

    void checkDirectionValidity()
    {
      if (type == FLIP) {
        if (door_direction.compare(DIRECTION_FLIP_CLOCKWISE) != 0 && door_direction.compare(DIRECTION_FLIP_COUNTER_CLOCKWISE) != 0) {
          ROS_WARN("Invalid door direction specified. Only two states possible: 'clockwise' OR 'counter_clockwise'. Defaulting to 'clockwise'");
          door_direction = DIRECTION_FLIP_CLOCKWISE;
        }
      } else if (type == SLIDE) {
        if (door_direction.compare(DIRECTION_SLIDE_LEFT) != 0 && door_direction.compare(DIRECTION_SLIDE_RIGHT) != 0) {
          ROS_WARN("Invalid door direction specified. Only two states possible: 'left' OR 'right'. Defaulting to 'left'");
          door_direction = DIRECTION_SLIDE_LEFT;
        }
      }
    }

    void determineModelDomain(sdf::ElementPtr _sdf)
    {
      if (!_sdf->HasElement("model_domain_space")) {
        ROS_WARN("Model Domain Space not specified in the plugin reference. Defaulting to 'door_'");
        model_domain_space = "door_";
      } else {
        model_domain_space = _sdf->GetElement("model_domain_space")->Get<std::string>();
      }

      ROS_INFO("Door '%s' initialized - Type: %s, Direction: %s, Domain Space: %s\n", door_type.c_str(), door_model_name.c_str(), door_direction.c_str(), model_domain_space.c_str());
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

      rosNode = new ros::NodeHandle("");

      sub = rosNode->subscribe<geometry_msgs::Twist>("/door_controller/command", 1000, &DoorPlugin::cmd_ang_cb, this );
      sub_active = rosNode->subscribe<std_msgs::UInt32MultiArray>("/door_controller/active", 1000, &DoorPlugin::active_doors_cb, this);

      updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&DoorPlugin::OnUpdate, this));
    }

    void cmd_ang_cb(const geometry_msgs::Twist::ConstPtr& msg)
    {
      if (isActive) {
        if (type == FLIP) {
          setAngularVel(msg->angular.z);
          ROS_INFO("Door '%s' - Angular z: [%f]", door_model_name.c_str(), msg->angular.z);
        } else if (type == SLIDE) {
          setLinearVel(msg->linear.x, msg->linear.y);
          ROS_INFO("Door '%s' - Linear x: [%f], y: [%f]", door_model_name.c_str(), msg->linear.x, msg->linear.y);
        }
      }
    }

    void updateLinkVel()
    {
      if (type == FLIP) {
        doorLink->SetAngularVel(cmd_vel);
      } else if (type == SLIDE) {
        doorLink->SetLinearVel(cmd_vel);
      }
    }

    void setAngularVel(float rot_z)
    {
      cmd_vel = math::Vector3();

      if (door_direction.compare(DIRECTION_FLIP_CLOCKWISE) == 0) { 
        cmd_vel.z = rot_z;
      } else {
        cmd_vel.z = -rot_z; 
      }
    }

    void setLinearVel(float lin_x, float lin_y) 
    {
      cmd_vel = math::Vector3();

      if (door_direction.compare(DIRECTION_SLIDE_LEFT) == 0) {
        cmd_vel.x = -lin_x;
        cmd_vel.y = -lin_y;
      } else {
        cmd_vel.x = lin_x;
        cmd_vel.y = lin_y;
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

    // Deprecated function:
    std::vector<std::string> parseTopicStr(std::string bot_pose_topics_str)
    {
      std::vector<std::string> bot_pose_topic_list;

      // parse csv-style input (also remove whitespace):
      std::string::iterator end_pos = std::remove(bot_pose_topics_str.begin(), bot_pose_topics_str.end(), ' ');
      bot_pose_topics_str.erase(end_pos, bot_pose_topics_str.end());

      std::istringstream ss(bot_pose_topics_str);
      std::string token;

      while (std::getline(ss, token, ',')) {
        bot_pose_topic_list.push_back(token.c_str());
      }

      return bot_pose_topic_list;
    }

  };

  GZ_REGISTER_MODEL_PLUGIN(DoorPlugin)
}