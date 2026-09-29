cmake_minimum_required(VERSION 3.8)
project(yolov8_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# Generate messages
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/StreamControl.msg"
  "msg/DetectControl.msg"
  "msg/Detection.msg"
  DEPENDENCIES std_msgs
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

# Install README for package-level usage and interface documentation.
install(FILES
  README.md
  DESTINATION share/${PROJECT_NAME}
)

# Install reusable Python modules next to executable scripts.
# This package also generates Python message modules named `yolov8_ros2.msg`;
# installing these helpers beside the scripts avoids conflicting with rosidl's
# generated Python package target while still supporting fallback local imports.
install(FILES
  yolov8_ros2/depth_utils.py
  yolov8_ros2/logging_utils.py
  yolov8_ros2/model_runner.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python executables
install(PROGRAMS
  yolov8_ros2/yolov8_infer_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME yolov8_infer_node
  PERMISSIONS OWNER_EXECUTE OWNER_READ GROUP_EXECUTE GROUP_READ WORLD_EXECUTE WORLD_READ
)
install(PROGRAMS
  yolov8_ros2/stream_control_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME stream_control_node
  PERMISSIONS OWNER_EXECUTE OWNER_READ GROUP_EXECUTE GROUP_READ WORLD_EXECUTE WORLD_READ
)
install(PROGRAMS
  yolov8_ros2/detect_control_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME detect_control_node
  PERMISSIONS OWNER_EXECUTE OWNER_READ GROUP_EXECUTE GROUP_READ WORLD_EXECUTE WORLD_READ
)

# Export ROS interfaces
ament_export_dependencies(rosidl_default_runtime)

ament_package()
