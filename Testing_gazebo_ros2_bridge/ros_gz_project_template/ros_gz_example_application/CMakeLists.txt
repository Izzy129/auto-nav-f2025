cmake_minimum_required(VERSION 3.5)

project(ros_gz_example_application)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

ament_package()

# --- GNSS Subscriber Node ---
# Defines the new gnss_subscriber executable. Updated Sept 25. 
# Add the executable for our subscriber node
add_executable(gnss_subscriber src/gnss_subscriber.cpp)

# Link the executable against its dependencies (rclcpp and sensor_msgs)
ament_target_dependencies(
  gnss_subscriber
  "rclcpp"
  "sensor_msgs"
)

# Install the executable so it can be found by ros2 run/launch
install(
  TARGETS gnss_subscriber
  DESTINATION lib/${PROJECT_NAME}
)