Dynamic Gazebo Models
==============

If your robotics application requires you to simulate automated doors/elevators in Gazebo, then this package has all the essentials tools you need. Currently available models: flip-open doors, slide-open doors, elevators with automatic slide-open doors (more coming soon). Plus, the bundle also comes with a generic dynamics-manager to control model groups through ROS service-calls or keyboard-op.

### Architecture

The dynamic properties of a model, e.g.:linear & angular velocities, are controlled via topics. These topics are wrapped with a layer of custom services to process commands such as 'open-close state' for flip-open doors  or 'target floor' for elevators.

## Installation

### Dependencies
ROS Hydro
Gazebo 3.0+

