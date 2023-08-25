# DH Gripper ROS

This repository is a fork of the original dh_gripper_ros package, featuring substantial enhancements and modifications for improved performance and functionality.

## New Features

- **Gripper Action Server**: We've introduced a powerful gripper action server that facilitates seamless control of the DH AG95 gripper through MoveIt. This integration streamlines gripper control and enhances compatibility.

- **Revamped URDF**: Our URDF model has undergone a significant overhaul. This includes a complete remodeling of both visual and collision meshes, resulting in a more accurate and realistic representation. The integration of new Gazebo support further enhances the overall experience.

- **Gazebo Integration**: The AG95 Gripper URDF now incorporates a sophisticated 4-bar linkage simulation within Gazebo. This provides a dynamic and interactive environment for testing and experimentation.

## Optional Mimic Joint Plugin

For users seeking an advanced feature, we offer an optional mimic joint plugin for synchronized control of both gripper fingers in Gazebo. To enable this feature, you can include the [roboticsgroup_upatras_gazebo_plugins](https://github.com/roboticsgroup/roboticsgroup_upatras_gazebo_plugins.git) repository in your workspace.

## Usage

To effectively utilize the gripper and its action server, execute the following command:

```bash
roslaunch dh_gripper_action_server dh_ag95_bringup.launch
```

For launching the Gazebo simulation of the gripper, use:

```bash
roslaunch dh_ag95_description gazebo.launch
```

To visualize the URDF in RViz, initiate the following:

```bash
roslaunch dh_ag95_description display.launch
```

We invite you to explore the enhanced capabilities of the DH AG95 gripper through this repository. Feel free to contribute, report issues, or provide feedback. Happy gripping!
