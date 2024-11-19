# xArm Plan Manager

## Overview

This repository provides a ROS2 package, `xarm_plan_manager`, for planning and executing a series of movements with the UFactory xArm robot using `xarm_planner` services. It allows users to perform both joint-based and pose-based movement sequences with retry mechanisms and scaling factors, supporting multiple degrees of freedom (DOF).

This package is designed to work with ROS2 Humble and integrates with the [xArm ROS2 repository](https://github.com/xArm-Developer/xarm_ros2/tree/humble).

## Features

- Supports joint-based and pose-based movement planning for xArm robots.
- Configurable DOF (5, 6, or 7), supporting different robot types (e.g., `xarm`, `lite`, `uf850`).
- Ability to set scaling factors to control the speed of movements.
- Retries for failed motion executions and timeout handling for reliability.

## Requirements

- ROS2 Humble
- [xarm\_ros2 Humble](https://github.com/xArm-Developer/xarm_ros2/tree/humble)
- UFactory xArm robot (but also works with the mockup hardware xarm\_planner launcher)

## Installation

1. Clone the repository into your ROS2 workspace:

   ```sh
   cd ~/ros2_ws/src
   git clone https://github.com/pastoriomarco/xarm_plan_manager.git
   ```

2. Install dependencies:

   ```sh
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. Build the package:

   ```sh
   cd ~/ros2_ws
   colcon build --packages-select xarm_plan_manager
   ```

4. Source the workspace:

   ```sh
   source install/setup.bash
   ```

## Usage

### Launching the Node

To launch the `plan_manager_node`, use the provided launch file. For example to launch a xarm6 planning example:

```sh
ros2 launch xarm_plan_manager plan_manager.launch.py robot_type:=xarm dof:=6
```

This command starts the plan manager for a 6 DOF `xarm`. You can adjust the `robot_type` and `dof` parameters accordingly:

- `robot_type`: `xarm`, `lite`, or `uf850`
- `dof`: 5, 6, or 7

### Example Movements

The `plan_manager_node` allows users to execute a series of pre-defined movements. These include both joint positions and Cartesian poses. Each movement can be executed with different speed scaling factors, allowing for more granular control of the motion execution.

Below are the main functions provided by the `PlanManager` class:

- **executeJointMovement(target\_joint\_positions, scaling\_factors)**: Plans and executes movement to the specified joint positions.
- **executePoseMovement(target\_pose, scaling\_factors)**: Plans and executes movement to the specified pose.

The movements are executed with a retry mechanism to ensure that the robot reaches the target, and scaling factors are used to control the speed.

### Example Commands

The following examples describe how the node executes a sequence of movements:

1. Move the robot to `tar_joint1` with a slow speed.
2. Move to `target_pose1` with a standard speed.
3. Continue with subsequent joint and pose targets.

Each movement uses a retry mechanism to increase reliability, ensuring the robot reaches the desired positions.

## Parameters

- **robot\_type**: The type of the robot (`xarm`, `lite`, `uf850`).
- **dof**: Degrees of Freedom (5, 6, or 7).
- **scaling\_factors**: A list of scaling factors for controlling the speed of motion.

## Contribution

Feel free to open issues or submit pull requests if you'd like to contribute to the project. Contributions for additional features, improvements, and bug fixes are always welcome.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

