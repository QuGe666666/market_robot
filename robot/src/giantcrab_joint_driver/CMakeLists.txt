cmake_minimum_required(VERSION 3.8)
project(giantcrab_joint_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =========================
# 基础依赖
# =========================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# =========================
# 生成自定义 srv 接口
# =========================
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetAngle.srv"
  "srv/GetStatus.srv"
  "srv/SetAngle.srv"
  "srv/SetLimits.srv"
  "srv/SetSpeed.srv"
)

# =========================
# 头文件目录
# 这里非常关键：让编译器能找到 include/controlcanfd.h
# =========================
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# =========================
# 可执行文件
# =========================
add_executable(giantcrab_joint_node
  src/giantcrab_joint_node.cpp
)

target_include_directories(giantcrab_joint_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(giantcrab_joint_node
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
  ament_index_cpp
)

# 你的程序里使用了 dlopen / dlsym
target_link_libraries(giantcrab_joint_node
  dl
)

# 让节点能使用本包生成的 srv 类型支持
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(giantcrab_joint_node "${cpp_typesupport_target}")

# =========================
# 选择并安装对应架构的第三方动态库
# 你的代码运行时会去包安装目录的 lib 下找 libcontrolcanfd.so
# 所以必须把 so 安装进去
# =========================
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  set(THIRD_PARTY_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/aarch64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
  set(THIRD_PARTY_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/x86_64)
else()
  message(FATAL_ERROR "不支持的架构: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

if(NOT EXISTS "${THIRD_PARTY_LIB_DIR}/libcontrolcanfd.so")
  message(FATAL_ERROR "未找到动态库: ${THIRD_PARTY_LIB_DIR}/libcontrolcanfd.so")
endif()

# 可执行文件安装
install(TARGETS
  giantcrab_joint_node
  DESTINATION lib/${PROJECT_NAME}
)

# 头文件安装
install(DIRECTORY include/
  DESTINATION include
)

# 配置 / launch / 文档安装
install(DIRECTORY config launch docs
  DESTINATION share/${PROJECT_NAME}
)

# README 安装
install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

# 安装第三方动态库和静态库到 install/lib
install(FILES
  ${THIRD_PARTY_LIB_DIR}/libcontrolcanfd.so
  DESTINATION lib
)

if(EXISTS "${THIRD_PARTY_LIB_DIR}/libcontrolcanfd.a")
  install(FILES
    ${THIRD_PARTY_LIB_DIR}/libcontrolcanfd.a
    DESTINATION lib
  )
endif()

# 如果你后面有别的包要 include 本包头文件，可保留这两项
ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)

ament_package()