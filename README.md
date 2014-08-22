Dynamic Gazebo Models
==============

Gazebo models for simulating doors/elevators. Currently available models: flip-open doors, slide-open doors, elevators with automatic slide-open doors (more coming soon). Plus, the bundle also comes with a generic dynamics-manager to control model groups through ROS service-calls or keyboard-op.

## Architecture

The dynamic properties of a model, e.g.:linear & angular velocities, are controlled via topics. These topics are wrapped with a layer of custom services to process commands such as 'open-close' for flip-open doors  or 'target floor' for elevators.

## Dependencies & Prerequisites
[ROS Hydro](http://wiki.ros.org/hydro), [Gazebo 3.0+](http://gazebosim.org/), 
[Catkin](http://wiki.ros.org/catkin): see [package.xml](package.xml)

## Installation
Clone and catkin_make
```bash
$ git clone https://github.com/MohitShridhar/dynamic_gazebo_models.git
$ cd <catkin_ws>
$ catkin_make --pkg dynamic_gazebo_models
```
Launch sample:
```bash
$ roslaunch dynamic_gazebo_models dynamic_models_test.launch
```
There are a lot of models to spawn, so be patient. You should see a bunch of doors and elevators:
![Flip-open, slide-open, elevators & auto-doors](images/models_screenshot.png)

## Usage

### Manual control
```bash
$ rosrun dynamic_gazebo_models keyboard_op 
```
Follow the instructions to control a group of doors | elevators.
### Service Calls
Include `dynamics_manager` node in your launch file (where you spawn your doors and elevators):
```xml
<node pkg="dynamic_gazebo_models" type="dynamics_manager" name="dynamics_manager" output="screen"/>
```

List of available services:
```bash
$ rosservice list
...
/model_dynamics_manager/add_control_group
/model_dynamics_manager/delete_control_group
/model_dynamics_manager/doors/open_close
/model_dynamics_manager/doors/set_vel
/model_dynamics_manager/elevators/open_close_elev
/model_dynamics_manager/elevators/set_props
/model_dynamics_manager/elevators/target_floor
/model_dynamics_manager/list_groups
...
```
Before you can send commands such as 'open doors', you need to create a group (of a single type; elevators or doors) specifying the desired units you want to control. This can be done during launch or runtime.

Add Group (**command-line**):
```bash
$ rosservice call /model_dynamics_manager/add_control_group "group: 
  group_name: 'New_group_of_doors'
  type: 'door'
  active_units: [1, 3, 4]"
```
This will add a new control group with doors 1, 3 & 4. *Hint:* Type `rosservice call /model_dynamics_manager/add_control_group` and press TAB to auto-complete the service-request.

Add Group (**launch file**):
```xml
<node pkg="rosservice" type="rosservice" name="all_doors" args="call 
$(arg add_group_srv)
'{ 
group: 
  {
    group_name: 'New_group_of_doors',
    type: 'door',
    active_units: [1, 3, 4]
  }
}'
"/>
```
Add Group (**cpp**):
```cpp
#include <dynamic_gazebo_models/AddGroup.h>
...
	ros::ServiceClient add_group_client = rosNode.serviceClient<dynamic_gazebo_models::AddGroup>("model_dynamics_manager/add_control_group");

    dynamic_gazebo_models::AddGroup addSrv;
    addSrv.request.group.group_name = "New_group_of_doors";
    addSrv.request.group.type = "door";
    addSrv.request.group.active_units = {1, 3, 4};
    
    add_group_client.call(addSrv);
...
```
Once you have setup your groups, you can use other services such as `/model_dynamics_manager/doors/open_close` in a similar fashion to control the dynamics of the model.

Flip-door - Open Doors:
```bash
rosservice call /model_dynamics_manager/doors/open_close "group_name: 'New_group_of_doors'
state: true"
```
To open doors 1, 3 & 4 simultaneously.

See [example launch file](launch/dynamic_models_test.launch) & [keyboard op](controllers/keyboard_op.cpp) for more sample implementations of the available service calls.

### Models
You may wish to modify the existing [models](models) or implement your own physical-object and still use the existing [plugins](src/plugins). Before embedding the custom models in your application, ensure that you have configured the SDF file properly.

#### Doors
```xml
<plugin name="door_plugin" filename="libdoor_plugin.so">
    <door_type>flip</door_type>
    <model_domain_space>door_</model_domain_space>
    <door_direction>clockwise</door_direction>
</plugin>
```
`door_type` can either be "flip" or "slide". `model_domain_space` is the prefix of the spawn model-name in Gazebo, eg: "door_1" --> prefix: "door_". `door_direction` can either be "counter_clockwise" or "clockwise" for flip-open doors, "left" or "right" for slide-open doors

#### Elevators
```xml
<plugin name="elevator_plugin" filename="libelevator.so">
    <model_domain_space>elevator_</model_domain_space>
    <floor_heights>0.84108, 3.65461, 6.85066, 10.0470, 13.24549, 16.45915, 19.65369</floor_heights>
    <speed>1.5</speed>
    <force>100</force>
</plugin>
```
`floor_heights` are absolute heights (in meters) of the desired floor levels. The values will be indexed (starting with 0 for the ground-floor) in ascending order. So 'target floor 2' in this case maps to a target height of 6.85066m. `speed` specifies the travel velocity of the elevator (in both directions; in m/s). `force` specifies the force exerted on the objects inside the elevator (in Newtons).

#### Auto-Doors
```xml
<plugin name="auto_elev_door_plugin" filename="libauto_door.so">
    <model_domain_space>auto_door_</model_domain_space>
    <elevator_name>elevator_1</elevator_name>
    <door_direction>left</door_direction>
    <max_trans_dist>0.711305</max_trans_dist>
    <speed>1.0</speed>
</plugin>
```
`elevator_name` is the spawn model-name of the elevator that controls the auto-door. `max_trans_dist` is maximum distance a sliding-door slides from it's spawned position before stopping.


