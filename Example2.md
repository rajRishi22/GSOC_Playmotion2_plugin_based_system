### Create a C++ ROS 2 node that sends some trajectory for a TIAGo arm on simulation

1. Source ROS 2 and Set Up Workspace
```
source /opt/ros/humble/setup.bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
```
2. Install TIAGo Simulation
 2.1 Install TIAGo Simulation Dependencies
 ```
sudo apt update
sudo apt install -y ros-humble-tiago-simulation ros-humble-gazebo-ros-pkgs ros-humble-ros2-control ros-humble-trajectory-msgs

 ```
 2.2 Launch TIAGo Simulation
 ```
 ros2 launch tiago_gazebo tiago_gazebo.launch.py is_public_sim:='True'

 ```
3. Create a ROS 2 Package for the TIAGo Arm Trajectory Node
3.1 Navigate to the src folder of your workspace and create a package named tiago_arm_trajectory with dependencies on rclcpp (for ROS 2) and trajectory_msgs (for sending joint trajectories).
```
cd ~/ros2_ws/src
ros2 pkg create tiago_arm_trajectory --build-type ament_cmake --dependencies rclcpp trajectory_msgs
```
4. Write the C++ Node
Navigate to the package folder:
`cd ~/ros2_ws/src/tiago_arm_trajectory`
Create a src directory and add a C++ source file:
```
mkdir src
nano src/send_trajectory.cpp
```
4.1 Add the Following Code in send_trajectory.cpp
```
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

class TIAGoArmTrajectory : public rclcpp::Node {
public:
    TIAGoArmTrajectory() : Node("tiago_arm_trajectory_publisher") {
        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/position_joint_trajectory_controller/joint_trajectory", 10);

        timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&TIAGoArmTrajectory::publish_trajectory, this));
    }

private:
    void publish_trajectory() {
        auto message = trajectory_msgs::msg::JointTrajectory();

        // Define joint names for the TIAGo arm
        message.joint_names = {
            "arm_1_joint", "arm_2_joint", "arm_3_joint",
            "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"};

        // Define a trajectory point
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = {0.2, -0.5, 0.3, 1.2, -1.0, 0.5, 0.8};  // Example joint values
        point.time_from_start = rclcpp::Duration::from_seconds(2.0);

        message.points.push_back(point);

        RCLCPP_INFO(this->get_logger(), "Publishing TIAGo arm trajectory...");
        publisher_->publish(message);
    }

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TIAGoArmTrajectory>());
    rclcpp::shutdown();
    return 0;
}
```
5. Modify CMakeLists.txt
Open CMakeLists.txt:
`nano CMakeLists.txt`
Replace its contents with the following:
```
cmake_minimum_required(VERSION 3.5)
project(tiago_arm_trajectory)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)

# Add the executable
add_executable(send_trajectory src/send_trajectory.cpp)
ament_target_dependencies(send_trajectory rclcpp trajectory_msgs)

# Install
install(TARGETS send_trajectory
    DESTINATION lib/${PROJECT_NAME})

ament_package()
```
6: Build the Package
```
cd ~/ros2_ws
colcon build --packages-select tiago_arm_trajectory
```
Source the workspace:
```
source install/setup.bash
```
7. Run the Node
```
ros2 launch tiago_gazebo tiago_gazebo.launch.py is_public_sim:='True'
```
Then, run your trajectory node:
```
ros2 run tiago_arm_trajectory send_trajectory

```

8. Verify Trajectory Execution
Open Gazebo and check if the TIAGo arm moves.

Use ROS 2 topic echo to see if the trajectory message is published:
```
ros2 topic echo /position_joint_trajectory_controller/joint_trajectory
```

