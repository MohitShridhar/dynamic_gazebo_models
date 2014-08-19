// Copyright (c) 2014 Mohit Shridhar, David Lee

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <stdio.h>
#include <map>
#include <math.h>

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <ros/ros.h>
#include <std_msgs/UInt32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>

#define DEFAULT_LIFT_SPEED 1.5
#define DEFAULT_LIFT_FORCE 100

#define UNKNOWN_FLOOR -100
#define HEIGHT_LEVEL_TOLERANCE 0.01

namespace gazebo
{   
  class ElevatorPlugin : public ModelPlugin
  {

    private: 

      ros::NodeHandle *rosNode;
      event::ConnectionPtr updateConnection;

      physics::ModelPtr model;
      physics::LinkPtr bodyLink;
      std::string modelName;

      ros::Subscriber target_floor_sub, active_elevs_sub, set_param_sub;
      ros::Publisher estimated_floor_pub;

      std::string model_domain_space, floor_heights_str;
      uint numFloors;

      std::map<int, float> floorHeightMap;
      std::map<float, int> floorIndexMap;

      bool isActive;
      int targetFloor, elev_ref_num;
      float elevSpeed, elevForce, spawnPosX, spawnPosY;

    public: 

      ElevatorPlugin()
      {
        std::string name = "elevator_plugin";
        int argc = 0;
        ros::init(argc, NULL, name);
      }

      ~ElevatorPlugin()
      {
        delete rosNode;
      }

      void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
      {
        establishLinks(_parent);
        detemineModelDomain(_sdf);
        loadFloorHeights(_sdf);
        loadSpeedForce(_sdf);
        initVars();
      }

    private:

      void OnUpdate()
      {
        ros::spinOnce();
        directElevator();
        constrainHorizontalMovement();
        publishEstimatedPos();
      }

      void detemineModelDomain(sdf::ElementPtr _sdf)
      {
        if (!_sdf->HasElement("model_domain_space")) {
          ROS_WARN("Model Domain Space not specified in the plugin reference. Defaulting to 'elevator_'");
          model_domain_space = "elevator_";
        } else {
          model_domain_space = _sdf->GetElement("model_domain_space")->Get<std::string>();
        }

        rosNode->setParam("/model_dynamics_manager/elevator_domain_space", model_domain_space);
      }

      void loadFloorHeights(sdf::ElementPtr _sdf)
      {
        if (!_sdf->HasElement("floor_heights")) {
          ROS_ERROR("Floor heights not specified in the plugin reference. The elevator model cannot function without known floor heights");
          std::exit(EXIT_FAILURE);
        } else {
          floor_heights_str = _sdf->GetElement("floor_heights")->Get<std::string>();
        }

        parseFloorHeights(floor_heights_str);

      }

      void loadSpeedForce(sdf::ElementPtr _sdf)
      {
        if (!_sdf->HasElement("speed")) {
          ROS_WARN("Elevator Speed not specified in the plugin reference. Defaulting to 1.5 m/s");
          elevSpeed = DEFAULT_LIFT_SPEED;
        } else {
          elevSpeed = _sdf->GetElement("speed")->Get<float>();
        }

        if (!_sdf->HasElement("force")) {
          ROS_WARN("Elevator Speed not specified in the plugin reference. Defaulting to 100 N");
          elevForce = DEFAULT_LIFT_FORCE;
        } else {
          elevForce = _sdf->GetElement("force")->Get<float>();
        }
      }

      void establishLinks(physics::ModelPtr _parent)
      {
        model = _parent;
        bodyLink = model->GetLink("body");
        modelName = model->GetName();

        rosNode = new ros::NodeHandle("");

        target_floor_sub = rosNode->subscribe<std_msgs::Int32>("/elevator_controller/target_floor", 100, &ElevatorPlugin::target_floor_cb, this);
        active_elevs_sub = rosNode->subscribe<std_msgs::UInt32MultiArray>("/elevator_controller/active", 100, &ElevatorPlugin::active_elevs_cb, this);
        set_param_sub = rosNode->subscribe<std_msgs::Float32MultiArray>("/elevator_controller/param", 100, &ElevatorPlugin::set_param_cb, this);
        estimated_floor_pub = rosNode->advertise<std_msgs::Int32>("/elevator_controller/" + modelName + "/estimated_current_floor", 100);

        updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ElevatorPlugin::OnUpdate, this)); 
      }

      void target_floor_cb(const std_msgs::Int32::ConstPtr& floorRef)
      {
        if (isActive && targetFloor != floorRef->data) {
          if (floorHeightMap.count(floorRef->data) == 0) {
            ROS_ERROR("Elevator %d: Floor %d does not exist!", elev_ref_num, floorRef->data);
            return;
          }

          targetFloor = floorRef->data;
          ROS_INFO("Elevator %d: Target Floor - %d", elev_ref_num, targetFloor);
        }
      }

      void active_elevs_cb(const std_msgs::UInt32MultiArray::ConstPtr& activeList)
      {
        isActive = false;

        for (std::vector<uint32_t>::const_iterator it = activeList->data.begin(); it != activeList->data.end(); ++it)
        {
          if (*it == elev_ref_num) {
            isActive = true;
          }
        }
      }

      void set_param_cb(const std_msgs::Float32MultiArray::ConstPtr& param)
      {
        if (isActive) {

          if (param->data[0] != elevSpeed) {
            ROS_INFO("Lift speed of '%s' set to: %f m/s\n", model->GetName().c_str(), param->data[0]);
          }

          if (param->data[1] != elevForce) {
            ROS_INFO("Lift force of '%s' set to: %f N\n", model->GetName().c_str(), param->data[1]);
          }

          elevSpeed = param->data[0];
          elevForce = param->data[1];   
        } 
      }

      void parseFloorHeights(std::string floor_heights_str)
      {
        std::vector<double> floor_heights;

        // parse csv-style input (also remove whitespace):
        std::string::iterator end_pos = std::remove(floor_heights_str.begin(), floor_heights_str.end(), ' ');
        floor_heights_str.erase(end_pos, floor_heights_str.end());

        std::istringstream ss(floor_heights_str);
        std::string token;

        
        float height;

        while (std::getline(ss, token, ',')) {
          
          try {
            height = std::stod(token);
          } catch (...) {
            ROS_ERROR("Invalid floor height %s", token.c_str());
            std::exit(EXIT_FAILURE);
          }

          floor_heights.push_back(height);
        }

        genFloorMap(floor_heights);
      }

      void genFloorMap(std::vector<double> floor_heights)
      {
        sort(floor_heights.begin(), floor_heights.end());

        for (int floorIndex = 0; floorIndex < floor_heights.size(); floorIndex++)
        {
           floorHeightMap[floorIndex] = floor_heights.at(floorIndex);
           floorIndexMap[floor_heights.at(floorIndex)] = floorIndex;

           ROS_INFO("Mapped Floor%d to height: %f", floorIndex, floor_heights.at(floorIndex));
        }

        numFloors = floor_heights.size();
        ROS_DEBUG("Total number of floors initialized: %d", numFloors);
      }

      std::string replaceSubstring(std::string &s, std::string toReplace, std::string replaceWith)
      {
        return(s.replace(s.find(toReplace), toReplace.length(), replaceWith));
      }
      void directElevator()
      {
        float targetHeight = floorHeightMap[targetFloor];
        float currentHeight = bodyLink->GetWorldCoGPose().pos.z;
        float heightDiff = currentHeight - targetHeight;

        if (heightDiff > HEIGHT_LEVEL_TOLERANCE || heightDiff < HEIGHT_LEVEL_TOLERANCE) {
          if (heightDiff > 0.0) {
            moveDown();
          } else {
            moveUp();
          }
        } else {
          stopMotion();
        }
      }

      void constrainHorizontalMovement()
      {
        math::Pose currPose = model->GetWorldPose();
        float currHeight = currPose.pos.z;

        math::Pose stabilizedPose;
        stabilizedPose.rot.x = stabilizedPose.rot.y = stabilizedPose.rot.z = 0;
        
        stabilizedPose.pos.x = spawnPosX;
        stabilizedPose.pos.y = spawnPosY;
        stabilizedPose.pos.z = currHeight;     

        model->SetWorldPose(stabilizedPose);
      }

      void publishEstimatedPos()
      {
        std_msgs::Int32 estimatedFloor;
        estimatedFloor.data = estimateCurrFloor();
        estimated_floor_pub.publish(estimatedFloor);
      }

      int estimateCurrFloor()
      {
        float currHeight = bodyLink->GetWorldCoGPose().pos.z;

        for (int i=0; i<numFloors; i++) {
          if (fabs(currHeight - floorHeightMap[i]) < HEIGHT_LEVEL_TOLERANCE) {
            return i;
          }
        } 
        
        return UNKNOWN_FLOOR;
      }

      void moveUp()
      {
        bodyLink->SetForce(math::Vector3(0, 0, elevForce));
        bodyLink->SetLinearVel(math::Vector3(0, 0, elevSpeed));
      }

      void moveDown()
      {
        bodyLink->SetForce(math::Vector3(0, 0, -elevForce));
        bodyLink->SetLinearVel(math::Vector3(0, 0, -elevSpeed));
      }

      void stopMotion()
      {
        bodyLink->SetForce(math::Vector3(0, 0, 0));
        bodyLink->SetLinearVel(math::Vector3(0, 0, 0));
      }

      void initVars()
      {
        isActive = false;
        targetFloor = 0;

        std::string elev_ref_num_str = model->GetName(); 
        replaceSubstring(elev_ref_num_str, model_domain_space, "");
        elev_ref_num = atoi(elev_ref_num_str.c_str());

        spawnPosX = bodyLink->GetWorldPose().pos.x;
        spawnPosY = bodyLink->GetWorldPose().pos.y;
      }

  };

  GZ_REGISTER_MODEL_PLUGIN(ElevatorPlugin)
}