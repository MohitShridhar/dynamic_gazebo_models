Dynamic Gazebo Models
==============

If your robotics application requires you to simulate automated doors/elevators in Gazebo, then this package has all the essentials tools you need. Currently available models: flip-open doors, slide-open doors, elevators with automatic slide-open doors (more coming soon). Plus, the bundle also comes with a generic dynamics-manager to control model groups through ROS service-calls or keyboard-op.

### Architecture

The dynamic properties of a model, e.g.:linear & angular velocities, are controlled via topics. These topics are wrapped with a layer of custom services to process commands such as 'open-close state' for flip-open doors  or 'target floor' for elevators.

### Dependencies & Prerequisites
[ROS Hydro](http://wiki.ros.org/hydro), [Gazebo 3.0+](http://gazebosim.org/)

### Installation
Clone and catkin_make
```
git clone https://github.com/MohitShridhar/dynamic_gazebo_models.git
cd <catkin_ws>
catkin_make --pkg dynamic_gazebo_models
```
Launch sample:
```
roslaunch dynamic_gazebo_models dynamic_models_test.launch
```
There are a lot of models to spawn, so be patient. After a while, you should see a bunch of doors and elevators:
![Flip-open, slide-open, elevators & auto-doors](images/models_screenshot.png)
