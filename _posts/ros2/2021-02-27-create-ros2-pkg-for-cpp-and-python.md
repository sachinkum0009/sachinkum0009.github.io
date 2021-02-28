---
layout: post
title: "Create ROS2 pkg to build c++ and python files"
author: sachin
categories: [ros2]
tags: [ros2, c++, python]
image: assets/images/robot.jpeg
description: "Create ROS2 pkg to build custom msgs, srv, C++ files with includes and Python scripts with modules."
featured: true
hidden: false
comments: true
---


<p>
The Robot Operating System (ROS) is a set of software libraries and tools for building robot applications. From drivers to state-of-the-art algorithms, and with powerful developer tools, ROS has what you need for your next robotics project. And it’s all open source.
</p>

<hr>

<p>
ROS2 is the next version of ROS which provides more features and deals with limitations from the previous version. ROS2 provides three types of build-depends as ament_cmake, ament_python, and cmake.

Recently I was working on a ROS2 project in which I was using ROS2 python to create the node. In this, I need to create a custom ROS2 msg-type. I searched for the tutorials and documents to create custom msgs in the ros2 pkg with build type ament_python.

But I didn’t find any way to build a custom msg in the python build type pkg.

So, to solve this problem, I created a new pkg in ros2 with build_type ament_cmake to build the custom messages. That’s how I solved the problem.

But that is not all. This is not how it should work. We can need to find a way to build the msgs, c++ files, and even python scripts. Luckily, I found this and will share the process.
</p>


1. Create a ros2 pkg inside your colcon_ws

```bash
$ ros2 create pkg my_ros2_pkg --build-type --ament-cmake
```

2. Build the pkg

```bash
$ colcon build
```

3. Create a custom msg

Make a folder msg in the ros2 pkg and add file Num.msg

```bash
int64 num
```

4. Create a Publisher in C++ in src

make a file ros2_publisher.cpp in the src folder

```cpp
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "my_ros2_pkg/msg/num.hpp"

using namespace std::chrono_literals;

class Ros2Publisher : public rclcpp::Node
{
	public:
		Ros2Publisher()
			: Node("ros2_publisher"), count_(0)
		{
			publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
			num_publisher_ = this->create_publisher<my_ros2_pkg::msg::Num>("num", 10);
			timer_ = this->create_wall_timer(
					500ms, std::bind(&Ros2Publisher::timer_callback, this));
		}

	private:
		void timer_callback()
		{
			auto message = std_msgs::msg::String();
			message.data = "Hello, world! " + std::to_string(count_++);
			auto num_msg = my_ros2_pkg::msg::Num();
			num_msg.num = 100;
			RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
			publisher_->publish(message);
			num_publisher_->publish(num_msg);
		}

		rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
		rclcpp::Publisher<my_ros2_pkg::msg::Num>::SharedPtr num_publisher_;
		size_t count_;
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Ros2Publisher>());
	rclcpp::shutdown();
	return 0;
}
```

5. Create a subscriber in Python in scripts

create a folder named same as pkg e.g. my_ros2_pkg and inside this folder create a file __init__.py and add the files you want to use as modules

make a file ros2_subscriber.py in the folder scripts

```py
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class Ros2Subscriber(Node):
    def __init__(self):
        super().__init__('ros2_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    ros2_subscriber = Ros2Subscriber()

    rclpy.spin(ros2_subscriber)

    ros2_subscriber.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

6. Modify package.xml

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
	<name>my_ros2_pkg</name>
	<version>0.0.0</version>
	<description>TODO: Package description</description>
	<maintainer email="sachinkum123567@gmail.com">hunter</maintainer>
	<license>TODO: License declaration</license>

	<buildtool_depend>ament_cmake</buildtool_depend>
	<buildtool_depend>ament_cmake_python</buildtool_depend>

	<depend>rclcpp</depend>
	<depend>rclpy</depend>


	<test_depend>ament_lint_auto</test_depend>
	<test_depend>ament_lint_common</test_depend>

	<build_depend>rosidl_default_generators</build_depend>

	<exec_depend>rosidl_default_runtime</exec_depend>

	<member_of_group>rosidl_interface_packages</member_of_group>


	<export>
		<build_type>ament_cmake</build_type>


	</export>
</package>
```

7. Modify CMakeList.xml

```txt
cmake_minimum_required(VERSION 3.5)
project(my_ros2_pkg)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(my_ros2_pkg REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Num.msg"
 )

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Include cpp "include" directory
include_directories(include)

# Create Cpp executable
add_executable(ros2_publisher src/ros2_publisher.cpp)
ament_target_dependencies(ros2_publisher rclcpp my_ros2_pkg)

# Install Cpp executables
install(TARGETS
	ros2_publisher
	DESTINATION lib/${PROJECT_NAME}
	)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
	scripts/ros2_subscriber.py
	DESTINATION lib/${PROJECT_NAME}
	)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

8. Test ros2 publisher node, subscriber node and custom msg

<b>Publisher</b>

```bash
$ ros2 run my_ros2_pkg ros2_publisher 
[INFO] [1614431609.679963831] [ros2_publisher]: Publishing: 'Hello, world! 0'
[INFO] [1614431610.180021664] [ros2_publisher]: Publishing: 'Hello, world! 1'
[INFO] [1614431610.679854568] [ros2_publisher]: Publishing: 'Hello, world! 2'
[INFO] [1614431611.179983858] [ros2_publisher]: Publishing: 'Hello, world! 3'
[INFO] [1614431611.679797258] [ros2_publisher]: Publishing: 'Hello, world! 4'
[INFO] [1614431612.179901033] [ros2_publisher]: Publishing: 'Hello, world! 5'
[INFO] [1614431612.679918919] [ros2_publisher]: Publishing: 'Hello, world! 6'
[INFO] [1614431613.179945430] [ros2_publisher]: Publishing: 'Hello, world! 7'
[INFO] [1614431613.679860337] [ros2_publisher]: Publishing: 'Hello, world! 8'
[INFO] [1614431614.179820863] [ros2_publisher]: Publishing: 'Hello, world! 9'
[INFO] [1614431614.679830158] [ros2_publisher]: Publishing: 'Hello, world! 10'
[INFO] [1614431615.179736326] [ros2_publisher]: Publishing: 'Hello, world! 11'
[INFO] [1614431615.679892254] [ros2_publisher]: Publishing: 'Hello, world! 12'
[INFO] [1614431616.179879969] [ros2_publisher]: Publishing: 'Hello, world! 13'
[INFO] [1614431616.679818267] [ros2_publisher]: Publishing: 'Hello, world! 14'
^C[INFO] [1614431617.169909878] [rclcpp]: signal_handler(signal_value=2)
```

<b>Subscriber</b>

```bash
$ ros2 run my_ros2_pkg ros2_subscriber.py 
[INFO] [1614431612.224064221] [ros2_subscriber]: I heard: "Hello, world! 5"
[INFO] [1614431612.683359069] [ros2_subscriber]: I heard: "Hello, world! 6"
[INFO] [1614431613.183585038] [ros2_subscriber]: I heard: "Hello, world! 7"
[INFO] [1614431613.682893150] [ros2_subscriber]: I heard: "Hello, world! 8"
[INFO] [1614431614.182802153] [ros2_subscriber]: I heard: "Hello, world! 9"
[INFO] [1614431614.681286602] [ros2_subscriber]: I heard: "Hello, world! 10"
[INFO] [1614431615.180778766] [ros2_subscriber]: I heard: "Hello, world! 11"
[INFO] [1614431615.683260114] [ros2_subscriber]: I heard: "Hello, world! 12"
[INFO] [1614431616.183141831] [ros2_subscriber]: I heard: "Hello, world! 13"
[INFO] [1614431616.683013687] [ros2_subscriber]: I heard: "Hello, world! 14"
```

<b>ROS2 custom msg </b>

```bash
$ ros2 topic echo /num
num: 100
---
num: 100
---
num: 100
---
num: 100
---
num: 100
---
num: 100
---
num: 100
---
```

<p>
That’s all! For ros2 pkg that can build custom msgs and srvs and even build C++ file with includes and Python scripts with modules. ❤️
</p>
