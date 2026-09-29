cmake_minimum_required(VERSION 3.8)
project(lh_chassis_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BatteryState.msg"
  "msg/ChassisCallback.msg"
  "msg/ChassisNotification.msg"
  "msg/ChassisStatus.msg"
  "msg/DeviceState.msg"
  "msg/OperationState.msg"
  "msg/PoseSpeed.msg"
  "msg/RobotState.msg"
  "msg/SceneInfo.msg"
  "msg/TaskProcess.msg"
  "srv/AccessiblePointQuery.srv"
  "srv/ApiCommand.srv"
  "srv/CheckSoftwareUpdate.srv"
  "srv/CountMarkers.srv"
  "srv/DeleteMarker.srv"
  "srv/DistanceProbe.srv"
  "srv/GetCurrentMap.srv"
  "srv/GetDiagnosisResult.srv"
  "srv/GetLiftStatus.srv"
  "srv/SetSpeedLimits.srv"
  "srv/GetParams.srv"
  "srv/GetPlannedPath.srv"
  "srv/GetPowerStatus.srv"
  "srv/GetRobotInfo.srv"
  "srv/GetSoftwareVersion.srv"
  "srv/InsertMarker.srv"
  "srv/InsertMarkerByPose.srv"
  "srv/ListMapInfo.srv"
  "srv/ListMaps.srv"
  "srv/MakePlan.srv"
  "srv/PositionAdjustMarker.srv"
  "srv/PositionAdjustPose.srv"
  "srv/QueryMarkerBrief.srv"
  "srv/QueryMarkerList.srv"
  "srv/RestartSoftware.srv"
  "srv/SetCurrentMap.srv"
  "srv/SetLedColor.srv"
  "srv/SetLedLuminance.srv"
  "srv/Shutdown.srv"
  "srv/TwistCommand.srv"
  "srv/UpdateSoftware.srv"
  "srv/WifiActiveConnection.srv"
  "srv/WifiConnect.srv"
  "srv/WifiDetailList.srv"
  "srv/WifiInfo.srv"
  "srv/WifiList.srv"
  "action/AutoDock.action"
  "action/CruiseMarkers.action"
  "action/MoveToMarker.action"
  "action/MoveToPose.action"
  DEPENDENCIES action_msgs builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
