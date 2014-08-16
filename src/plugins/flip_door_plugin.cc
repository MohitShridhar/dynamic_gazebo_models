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
    std::string bot_pose_topics_str;

    ros::NodeHandle* rosNode;
    transport::NodePtr gazeboNode;
    event::ConnectionPtr updateConnection;

    std::vector<ros::Subscriber> bot_pose_subs;
    uint numBotsInContextSpace;

    transport::SubscriberPtr subFpvPose, subGzRequest;
    ros::Subscriber sub, sub_active;

    boost::mutex cb_mutex;
    boost::mutex::scoped_lock *lock;

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
      setupBotPoseSubscribers(_sdf);
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

    void setupBotPoseSubscribers(sdf::ElementPtr _sdf)
    {
      if (!_sdf->HasElement("bot_pose_topics")) {
        ROS_ERROR("Bot Pose subscription topics not specified. Defaulting to botPose");
        bot_pose_topics_str = "/botPose";
      } else {
        bot_pose_topics_str = _sdf->GetElement("bot_pose_topics")->Get<std::string>();
      }

      std::vector<std::string> bot_pose_topic_list = parseTopicStr(bot_pose_topics_str);

      for (int i=0; i<bot_pose_topic_list.size(); i++) {
        ros::Subscriber pose_sub = rosNode->subscribe<geometry_msgs::Pose>(bot_pose_topic_list.at(i), 2, &FlipDoor::bot_pose_cb, this);
        ROS_INFO("Subscribed to bot pose topic - '%s'", pose_sub.getTopic().c_str());
        bot_pose_subs.push_back(pose_sub);
      }

    }

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

    void initVars()
    {
      isActive = false;

      std::string door_ref_num_str = door_model_name; 
      replaceSubstring(door_ref_num_str, model_domain_space, "");
      door_ref_num = atoi(door_ref_num_str.c_str());

      numBotsInContextSpace = 0;
      lock = new boost::mutex::scoped_lock(cb_mutex);
    }

    void establishLinks(physics::ModelPtr _parent)
    {
      model = _parent;
      doorLink = model->GetLink("door");
      door_model_name = model->GetName();

      rosNode = new ros::NodeHandle("");

      sub = rosNode->subscribe<geometry_msgs::Twist>("/door_controller/command", 1000, &FlipDoor::cmd_ang_cb, this );
      sub_active = rosNode->subscribe<std_msgs::UInt32MultiArray>("/door_controller/active", 1000, &FlipDoor::active_doors_cb, this);

      updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&FlipDoor::OnUpdate, this));
    }

    void cmd_ang_cb(const geometry_msgs::Twist::ConstPtr& msg)
    {
      ROS_ASSERT(numBotsInContextSpace >= 0);

      if (isActive) {
        
        if (numBotsInContextSpace > 0) {
          setAngularVel(DEFAULT_OPEN_VEL);
        } else if (numBotsInContextSpace == 0) {
          setAngularVel(msg->angular.z);
          ROS_INFO("Door '%s' - Angular z: [%f]", door_model_name.c_str(), msg->angular.z);
        }

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

    void bot_pose_cb(const geometry_msgs::Pose::ConstPtr& msg) 
    {
      if (isObjectInContextSpace(*msg)) {
        // Use mutex lock to ensure race-condition doesn't affect the var 'numBotsInContextSpace'
        // lock->lock();
        numBotsInContextSpace++;
        // lock->unlock();

      } else {

        if (numBotsInContextSpace > 0) {
        
          // lock->lock();
          numBotsInContextSpace--;
          // lock->unlock();
        
        }
      }      

    }

    bool isObjectInContextSpace(geometry_msgs::Pose botPose)
    {
      math::Pose currDoorPose = this->model->GetWorldPose();

      if (botPose.position.x > currDoorPose.pos.x + CONTEXT_SPACE_X_RANGE || botPose.position.x < currDoorPose.pos.x - CONTEXT_SPACE_X_RANGE) {
        return false;
      }

      if (botPose.position.y > currDoorPose.pos.y + CONTEXT_SPACE_Y_RANGE || botPose.position.y < currDoorPose.pos.y - CONTEXT_SPACE_Y_RANGE) {
        return false;
      }

      if (botPose.position.z > currDoorPose.pos.z + CONTEXT_SPACE_Z_RANGE || botPose.position.z < currDoorPose.pos.z - CONTEXT_SPACE_Z_RANGE) {
        return false;
      }

      return true;
    }


    std::string replaceSubstring(std::string &s, std::string toReplace, std::string replaceWith)
    {
      return(s.replace(s.find(toReplace), toReplace.length(), replaceWith));
    }

  };

  GZ_REGISTER_MODEL_PLUGIN(FlipDoor)
}