# ROS2 Projects 🚀

Welcome to the **ROS2_projects** repository! 🎉 This repo contains two exciting ROS 2-based robotics projects: **Corbis** 🤖, an autonomous navigation stack for in-plant logistics robots, and **Line Follower** 🛤️, a simple yet effective line-following robot using proportional control. Whether you're navigating complex industrial environments or following lines with precision, this repo has you covered! 😎

## Table of Contents
- [Overview](#overview-)
- [Prerequisites](#prerequisites-)
- [Installation](#installation-)
- [Directory Structure](#directory-structure-)
- [Running the Corbis Navigation Stack](#running-the-corbis-navigation-stack-)
- [Running the Line Follower](#running-the-line-follower-)
- [Using the GUI for Parameter Tuning](#using-the-gui-for-parameter-tuning-)
- [Contributing](#contributing-)
- [License](#license-)

## Overview 🌟
This repository houses two ROS 2 projects developed with ROS 2 Jazzy Jalisco on Ubuntu 24.04:

1. **Corbis** 🤖: A robust navigation stack for Autonomous Mobile Robots (AMRs) designed for in-plant logistics, as detailed in the research paper *"A Simulation-Validated Autonomous Navigation Stack for In-Plant Logistics Robots using ROS 2"*. Built with the ROS 2 Navigation Stack (Nav2), it features:
   - 📍 **SLAM**: Graph-based mapping with `slam_toolbox`.
   - 📍 **Localization**: Adaptive Monte Carlo Localization (AMCL).
   - 📍 **Planning**: A* global planner and DWB local planner for smooth navigation and obstacle avoidance.
   - 📍 **Simulation**: A Gazebo-based industrial environment with narrow aisles and dynamic obstacles.
   - 📍 **Performance**: 95% mission success rate and 0.05m path tracking accuracy after tuning.

2. **Line Follower** 🛤️: A line-following robot using a proportional (P) control algorithm. It processes sensor data (e.g., IR sensors) to stay on track, perfect for learning ROS 2 and control systems.

## Prerequisites 🛠️
To run these projects, ensure you have:
- **OS**: Ubuntu 24.04 LTS 🐧
- **ROS 2**: Jazzy Jalisco 🌟
- **Dependencies**:
  - `ros-jazzy-nav2-bringup`
  - `ros-jazzy-slam-toolbox`
  - `ros-jazzy-gazebo-ros-pkgs`
  - `ros-jazzy-rqt-reconfigure`
  - `python3-colcon-common-extensions`
- **Hardware** (for simulation):
  - CPU: Quad-core or better ⚡
  - RAM: 8GB or more 💾
  - GPU: Recommended for Gazebo 🖼️

## Installation 📦
1. **Install ROS 2 Jazzy**:
   Follow the [ROS 2 Jazzy installation guide](https://docs.ros.org/en/jazzy/Installation.html).

2. **Install Dependencies**:
   ```bash
   sudo apt update
   sudo apt install ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox ros-jazzy-rqt-reconfigure ros-jazzy-gazebo-ros-pkgs python3-colcon-common-extensions
   ```

3. **Clone the Repository**:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   git clone <repository-url> ROS2_projects
   ```

4. **Build the Workspace**:
   ```bash
   cd ~/ros2_ws
   colcon build --symlink-install
   source install/setup.bash
   ```

## Directory Structure 📂
```
ROS2_projects/
├── corbis/                        # AMR navigation stack
│   ├── launch/                    # Launch files for Gazebo and Nav2
│   ├── urdf/                      # Differential drive robot URDF
│   ├── worlds/                    # Gazebo industrial environment
│   ├── config/                    # Nav2, SLAM, and AMCL configs
│   └── maps/                      # Generated 2D maps
├── lline_follower/                # Line-following robot
│   ├── scripts/                   # Python scripts for P-control
│   ├── launch/                    # Launch files for simulation/hardware
│   └── config/                    # P-control parameters
├── README.md                      # This file 📖
└── LICENSE                        # License file 📜
```

## Running the Corbis Navigation Stack 🚚
1. **Launch Gazebo Simulation**:
   ```bash
   ros2 launch corbis gazebo_simulation.launch.py
   ```
   This spawns the differential drive robot with LiDAR and IMU in a Gazebo industrial environment. 🌍

2. **Generate a Map**:
   ```bash
   ros2 launch corbis slam.launch.py
   ```
   Teleoperate the robot using:
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
   Save the map in `corbis/maps/` using `slam_toolbox`. 🗺️

3. **Run Navigation**:
   ```bash
   ros2 launch corbis navigation.launch.py
   ```
   Use RViz to set navigation goals. The stack uses A*, DWB, and AMCL for robust navigation. 🚀

4. **Tune Parameters**:
   See [Using the GUI for Parameter Tuning](#using-the-gui-for-parameter-tuning-) for live tuning with `rqt_reconfigure`. ⚙️

## Running the Line Follower 🛤️
1. **Launch the Line Follower**:
   ```bash
   ros2 launch lline_follower line_follower.launch.py
   ```
   This runs the line-following bot in Gazebo (if simulated) or on a physical robot with line sensors. 🔍

2. **P-Control Logic**:
   The bot follows a line using proportional control:
   ```python
   error = line_position - target_position  # Deviation from line
   correction = kp * error                  # Proportional correction
   left_speed = base_speed + correction     # Adjust motor speeds
   right_speed = base_speed - correction
   ```
   Tune the `kp` gain in `lline_follower/config/params.yaml`. 🎛️

3. **Simulation vs. Hardware**:
   - **Simulation**: Ensure the Gazebo world has a visible line (e.g., black on white). 🖼️
   - **Hardware**: Calibrate IR sensors for the physical line (e.g., black tape on white surface). ⚙️

## Using the GUI for Parameter Tuning 🎨
For **Corbis**:
1. Launch `rqt_reconfigure`:
   ```bash
   ros2 run rqt_reconfigure rqt_reconfigure
   ```
2. Select `/dwb_controller` to tune parameters like `max_vel_x`, `max_vel_theta`, `heading`, `dist`, and `velocity` weights.
3. Monitor path tracking in RViz, aiming for ~0.05m cross-track error. 📏
4. Save tuned parameters to `corbis/config/dwb_params.yaml`. 💾

For **Line Follower**:
- Edit `kp` in `lline_follower/config/params.yaml` manually or extend to ROS 2 parameters for GUI tuning. 🛠️

## Contributing 🤝
We love contributions! To get started:
1. Fork the repo 🍴
2. Create a branch: `git checkout -b my-feature` 🌿
3. Commit changes: `git commit -m "Add cool feature"` 💬
4. Push: `git push origin my-feature` 🚀
5. Open a pull request 📬

Follow ROS 2 style guidelines and include tests where possible. 😊

## License 📜
This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details. 🎉