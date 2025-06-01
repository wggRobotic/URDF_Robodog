# URDF_Robodog

This package provides the URDF (Unified Robot Description Format) model for the Robodog robot, including all necessary files to visualize and simulate the robot in ROS 2 using RViz.

## Features

- Complete URDF and Xacro description of the Robodog robot
- Launch files for easy visualization in RViz
- Example RViz configuration for convenient robot viewing

## Installation

Clone this repository into your ROS 2 workspace's `src` directory:

```sh
cd ~/ros2_ws/src
git clone <repository-url> urdf_robodog
```
Install dependencies and build the workspace:
```sh
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

## Usage
To visualize the Robodog model in Rviz, use the provided launch file:

```sh
ros2 launch urdf_robodog [display.launch.py](http://_vscodecontentref_/0)
```

This will start RViz with the Robodog model loaded and ready for inspection.

## File Structure
- urdf/ — Contains the URDF and Xacro files describing the robot
- launch/ — Launch files for visualization
- rviz/ — Example RViz configuration
## Dependencies
- ROS 2 (tested with Humble and newer)
- urdf_launch package

