## Example 3: Write a basic plugin-based system in C++ that uses ROS 2 Humble and create two plugins for that system. The plugin system will interact with strings, one plugin will revert the string and the other will count the number of characters. The user should be able to select which plugin to use.
## What is a Plugin-Based System ?
A plugin-based system allows you to dynamically load and use different modules (plugins) without modifying the main program. In our case:

The main node will let the user choose between two plugins:

- ReversePlugin → Reverses the given string.

- CountPlugin → Counts the number of characters in the string.

We will use ROS 2 plugins with the pluginlib library to create and manage these plugins.

##  Steps to Build the System

Create a ROS 2 package for the plugin system.

Define a common interface for plugins.

Implement two plugins (reverse and count).

Modify CMakeLists.txt and package.xml to support plugins.

Create a main node that dynamically loads a selected plugin.

Build and run the system.
### Step 1: Create a New ROS 2 Package
Open a terminal and navigate to your ROS 2 workspace:
```
cd ~/ros2_ws/src
ros2 pkg create string_plugin_system --build-type ament_cmake --dependencies rclcpp pluginlib
```
This creates a package named string_plugin_system.

Navigate into it by running `cd string_plugin_system`

### Step 2: Define the Plugin Interface
The plugin interface ensures that all plugins have a common structure.

Create an include directory:
```
mkdir include
mkdir include/string_plugin_system
```
Create a new header file `string_plugin_base.hpp` inside string_plugin_system for the interface with the following content:
```#ifndef STRING_PLUGIN_SYSTEM_STRING_PLUGIN_BASE_HPP_
#define STRING_PLUGIN_SYSTEM_STRING_PLUGIN_BASE_HPP_

#include <string>

class StringPluginBase
{
public:
    virtual ~StringPluginBase() = default;

    // Function that each plugin must implement
    virtual std::string processString(const std::string &input) = 0;
};

#endif
```
## Step 3: Implement the Plugins
### 3.1: Reverse Plugin
Create a src/plugins directory using `mkdir -p src/plugins`
Create a new file for the reverse plugin using `nano src/plugins/reverse_plugin.cpp` , add the follwing code
```
#include "string_plugin_system/string_plugin_base.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <algorithm>

class ReversePlugin : public StringPluginBase
{
public:
    std::string processString(const std::string &input) override
    {
        std::string reversed = input;
        std::reverse(reversed.begin(), reversed.end());
        return reversed;
    }
};

// Export the plugin
PLUGINLIB_EXPORT_CLASS(ReversePlugin, StringPluginBase)
```
## 3.2: Count Plugin
Create another file for the count plugin `nano src/plugins/count_plugin.cpp`
`count_plugin.cpp`
```
#include "string_plugin_system/string_plugin_base.hpp"
#include <pluginlib/class_list_macros.hpp>

class CountPlugin : public StringPluginBase
{
public:
    std::string processString(const std::string &input) override
    {
        return "Character count: " + std::to_string(input.size());
    }
};

// Export the plugin
PLUGINLIB_EXPORT_CLASS(CountPlugin, StringPluginBase)
```
## Step 4: Create the Main Node
Now, create a node that loads the plugin dynamically.
`nano src/main_node.cpp`
`main_node.cpp`
```
#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>
#include "string_plugin_system/string_plugin_base.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("plugin_node");

    // Ask the user which plugin to use
    std::string plugin_name;
    std::cout << "Enter plugin name (ReversePlugin or CountPlugin): ";
    std::cin >> plugin_name;

    try
    {
        // Load the selected plugin
        pluginlib::ClassLoader<StringPluginBase> loader("string_plugin_system", "StringPluginBase");
        std::shared_ptr<StringPluginBase> plugin = loader.createSharedInstance(plugin_name);

        // Get input string
        std::string input;
        std::cout << "Enter a string: ";
        std::cin.ignore();
        std::getline(std::cin, input);

        // Process and display result
        std::string result = plugin->processString(input);
        std::cout << "Result: " << result << std::endl;
    }
    catch (const pluginlib::PluginlibException &e)
    {
        std::cerr << "Error loading plugin: " << e.what() << std::endl;
    }

    rclcpp::shutdown();
    return 0;
}
```
## Step 5: Modify CMakeLists.txt
Open : `nano CMakeLists.txt`
Replace everything with :
```
cmake_minimum_required(VERSION 3.5)
project(string_plugin_system)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)

add_library(reverse_plugin SHARED src/plugins/reverse_plugin.cpp)
ament_target_dependencies(reverse_plugin pluginlib)

add_library(count_plugin SHARED src/plugins/count_plugin.cpp)
ament_target_dependencies(count_plugin pluginlib)

add_executable(main_node src/main_node.cpp)
ament_target_dependencies(main_node rclcpp pluginlib)

install(TARGETS main_node reverse_plugin count_plugin
        DESTINATION lib/${PROJECT_NAME})

ament_package()
```

## Step 6: Modify package.xml
Open `nano package.xml`
Ensure it contains:
```
<depend>rclcpp</depend>
<depend>pluginlib</depend>
```

##  Step 7: Build the Package

Navigate to your workspace and build:
```
cd ~/ros2_ws/src/string_plugin_system/include
colcon build --packages-select string_plugin_system
```
If there are errors, clean and rebuild:
`colcon build --packages-select string_plugin_system --clean`
Source the workspace: `source install/setup.bash`
## Step 8: Run the Plugin System
use command `ros2 run string_plugin_system main_node`
- ## Example Run
```
Enter plugin name (ReversePlugin or CountPlugin): ReversePlugin
Enter a string: hello
Result: olleh
```
```
Enter plugin name (ReversePlugin or CountPlugin): CountPlugin
Enter a string: hello
Result: Character count: 5
```
