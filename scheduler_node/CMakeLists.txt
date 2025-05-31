cmake_minimum_required(VERSION 3.8)
project(scheduler_node)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Build RoundRobinTaskScheduler
add_executable(roundrobintaskscheduler src/roundrobintaskscheduler.cpp)
ament_target_dependencies(roundrobintaskscheduler
  rclcpp
  std_msgs
)
target_include_directories(roundrobintaskscheduler PUBLIC /opt/ros/jazzy/include)

# Build DummyTaskNoderr
add_executable(dummytasknoderr src/dummytasknoderr.cpp)
ament_target_dependencies(dummytasknoderr
  rclcpp
  std_msgs
)
target_include_directories(dummytasknoderr PUBLIC /opt/ros/jazzy/include)

# Build DummyTaskNodeFCFS
add_executable(dummytasknodefcfs src/dummytasknodefcfs.cpp)
ament_target_dependencies(dummytasknodefcfs
  rclcpp
  std_msgs
)
target_include_directories(dummytasknodefcfs PUBLIC /opt/ros/jazzy/include)

# Build FCFSScheduler
add_executable(fcfsscheduler src/fcfsscheduler.cpp)
ament_target_dependencies(fcfsscheduler
  rclcpp
  std_msgs
)
target_include_directories(fcfsscheduler PUBLIC /opt/ros/jazzy/include)

# Build PriorityScheduler
add_executable(priorityscheduler src/priorityscheduler.cpp)
ament_target_dependencies(priorityscheduler
  rclcpp
  std_msgs
)
target_include_directories(priorityscheduler PUBLIC /opt/ros/jazzy/include)

# Build DummyTaskNodePriority
add_executable(dummytasknodepriority src/dummytasknodepriority.cpp)
ament_target_dependencies(dummytasknodepriority
  rclcpp
  std_msgs
)
target_include_directories(dummytasknodepriority PUBLIC /opt/ros/jazzy/include)

# Install all executables
install(TARGETS
  roundrobintaskscheduler
  dummytasknoderr
  dummytasknodefcfs
  fcfsscheduler
  priorityscheduler
  dummytasknodepriority
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()