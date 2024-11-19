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

   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/pastoriomarco/xarm_plan_manager.git
