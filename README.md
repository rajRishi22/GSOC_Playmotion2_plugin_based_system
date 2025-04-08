# GSOC_Playmotion2 Plugin Based System

This repository contains all the required information and code for the Playmotion2 plugin-based system project example.

## Table of Contents
- [Overview](#overview)
- [Examples](#examples)
- [Installation](#installation)
- [Usage](#usage)
- [Contact Details](#contact-details)
- [License](#license)

## Overview
This project demonstrates various aspects of ROS 2 Humble using different examples:
- A basic publish/subscribe example.
- A C++ node that publishes a trajectory for a TIAGo arm simulation.
- A plugin-based system in C++ that supports dynamic selection of plugins to process strings.

## Examples

1. **Publish/Subscribe Example**  
   A basic example covering ROS 2's publish/subscribe mechanism.

2. **TIAGo Arm Trajectory Publisher**  
   A ROS 2 node (written in C++) for sending trajectories to a TIAGo arm simulation.  
   Related repository: [TIAGo Simulation](https://github.com/pal-robotics/tiago_simulation)

3. **Plugin-based System**  
   Implements a system with a dynamic plugin selection using ROS 2 Humble.  
   - **String Reverser Plugin**: Reverses the provided string.  
   - **Character Counter Plugin**: Counts the number of characters in the string.

## Installation

1. **Clone the Repository**
   ```bash
   git clone <repository-url>
   cd GSOC_Playmotion2_plugin_based_system
   ```

2. **Set Up ROS 2 Environment**  
   Make sure you have ROS 2 Humble installed and source its setup file:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

3. **Build the Workspace**
   ```bash
   cd ros2_ws
   colcon build --symlink-install
   ```

4. **Source the Workspace**
   ```bash
   source install/setup.bash
   ```

## Usage

### Running the TIAGo Arm Trajectory Node
```bash
ros2 run tiago_arm_trajectory send_trajectory
```

### Running the Plugin-based System
Replace `<plugin_based_system_package>` and `<executable_name>` with your actual package and executable names:
```bash
ros2 run <plugin_based_system_package> <executable_name>
```

## Contact Details
For further information, please refer to [Contact Details](./Contact_details.md).

## License
This project is licensed under the MIT License. See the [LICENSE](./LICENSE) file for details.

