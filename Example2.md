### Create a C++ ROS 2 node that sends some trajectory for a TIAGo arm on simulation

1. Set Up Workspace and clone tiago_simulation:
```
mkdir -p ~/tiago_ws/src
cd ~/tiago_ws/src
git clone https://github.com/pal-robotics/tiago_simulation.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```
2. Launch the simulation
```
ros2 launch tiago_gazebo tiago_gazebo.launch.py is_public_sim:=True
```
 3. Create Your C++ Package
```
cd ~/tiago_ws/src
ros2 pkg create example_trajectory_publisher --build-type ament_cmake --dependencies rclcpp trajectory_msgs

```
after that move to example_trajectory_publisher:
```
cd example_trajectory_publisher
```

Add a node src/publish_trajectory.cpp inside example_trajectory/src:
```
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

class TrajectoryPublisher : public rclcpp::Node {
public:
  TrajectoryPublisher() : Node("trajectory_publisher") {
    publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/arm_controller/commands", 10);

    auto timer_callback = [this]() -> void {
      trajectory_msgs::msg::JointTrajectory traj;
      traj.joint_names = {"arm_1_joint", "arm_2_joint", "arm_3_joint", 
                          "arm_4_joint", "arm_5_joint", "arm_6_joint", 
                          "arm_7_joint"};

      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.positions = {0.0, -0.5, 0.0, -1.0, 0.0, 1.0, 0.0};
      point.time_from_start = rclcpp::Duration::from_seconds(3.0);

      traj.points.push_back(point);
      publisher_->publish(traj);
      RCLCPP_INFO(this->get_logger(), "Trajectory published.");
    };

    timer_ = this->create_wall_timer(std::chrono::seconds(5), timer_callback);
  }

private:
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryPublisher>());
  rclcpp::shutdown();
  return 0;
}
```
 4. Update CMakeLists.txt
Add this to the bottom of CMakeLists.txt:
```
add_executable(trajectory_publisher src/publish_trajectory.cpp)
ament_target_dependencies(trajectory_publisher rclcpp trajectory_msgs)
install(TARGETS
  trajectory_publisher
  DESTINATION lib/example_trajectory_publisher)
```
5. Build the package
```
cd ~/tiago_ws
colcon build
source install/setup.bash
```

 6. Run your node
 Make sure simulation is running, then:
```
ros2 run example_trajectory_publisher trajectory_publisher
```
If you want to check join names or controller :
To check available controllers:
```
ros2 control list_controllers
```
To verify joint names:
```
ros2 topic echo /joint_states
```