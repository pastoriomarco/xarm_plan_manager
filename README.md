# xArm Plan Manager
A ROS2 repository to plan and execute a series of movements with Ufactory robot through xarm_planner services

![xArm Logo](https://github.com/pastoriomarco/xarm_plan_manager/raw/main/assets/xarm_logo.png)

**xArm Plan Manager** is a ROS 2 package designed to manage and execute movement plans for [xArm](https://www.ufactory.cc/xarm) robotic arms. Leveraging the powerful `xarm_planner` from the [xarm_ros2](https://github.com/ufactory/xarm_ros2) repository, this package provides a robust framework for executing joint and pose movements with retry logic and scaling factors.

## Table of Contents

- [Features](#features)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)
  - [Launch File](#launch-file)
  - [Parameters](#parameters)
- [Node Overview](#node-overview)
- [Example Movements](#example-movements)
- [Contributing](#contributing)
- [License](#license)
- [Contact](#contact)
- [Acknowledgements](#acknowledgements)

## Features

- **Joint Movement Execution:** Move the robot arm to specified joint positions with configurable scaling factors and retry mechanisms.
- **Pose Movement Execution:** Move the robot arm's end-effector to specified poses with precision and reliability.
- **Scalable Design:** Supports robots with 5, 6, or 7 degrees of freedom (DOF).
- **Retry Logic:** Automatically retries movement executions upon failure, enhancing reliability.
- **Timeout Handling:** Configurable timeouts for movement completion to ensure timely operations.
- **Thread-Safe Joint State Management:** Ensures safe access to joint states across multiple threads.
- **Flexible Scaling Factors:** Allows dynamic adjustment of movement scaling factors to control speed and precision.

## Prerequisites

- **ROS 2:** Ensure that you have ROS 2 [Foxy](https://docs.ros.org/en/foxy/Installation.html) or later installed.
- **xArm ROS 2 Packages:** This package relies on the [xarm_ros2](https://github.com/ufactory/xarm_ros2) repository. Ensure that it is installed and properly configured.
- **C++17 Compiler:** Required for building the package.

## Installation

1. **Clone the Repository:**

   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/pastoriomarco/xarm_plan_manager.git
