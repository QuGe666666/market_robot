# Current System Audit

Audit date: 2026-08-28

This document records the interfaces used by the competition console. `CODE-VERIFIED`
means the interface and type were confirmed in the source tree. It does not mean the
corresponding hardware was running during this audit.

## Packages used as the integration baseline

| Package | Role | Audit status |
| --- | --- | --- |
| `supermarket_grasp_ui` | RGB-D/Qwen/YOLO/grasp control and the new competition console | CODE-VERIFIED |
| `supermarket_grasp_ros2` | Qwen bbox to GraspNet pose candidates | CODE-VERIFIED |
| `supermarket_pick_sequence` | Complete three-phase dual-arm Competition FSM and A-G navigation integration | CODE-VERIFIED |
| `curobo_realman_test` | Resident staged CuRobo planner | CODE-VERIFIED |
| `qwen2_5_vl_ros2` | Wrist-camera Qwen2.5-VL perception | CODE-VERIFIED |
| `yolov8_ros2` | YOLO stream/detection control | CODE-VERIFIED |
| `omnipicker_gripper` | Left/right OmniPicker ROS services | CODE-VERIFIED |
| `rm_driver` | RM65 arms and lift command/result topics | CODE-VERIFIED |
| `lh_chassis_bridge` / `chassis_ros` | Marker-based chassis navigation | CODE-VERIFIED |

## Current real interfaces

### Competition and navigation

| Interface | Type | Console use | Status |
| --- | --- | --- | --- |
| `/competition/status` | `std_msgs/msg/String` JSON | Preferred full telemetry/status | CODE-VERIFIED |
| `/competition/start` `/pause` `/resume` `/stop` `/reset` | `std_srvs/srv/Trigger` | Competition FSM controls | CODE-VERIFIED |
| `/supermarket_pick/status` `/competition_start` `/cancel` | existing compatibility interfaces | Legacy clients | CODE-VERIFIED |
| `/chassis/move_to_marker` | `lh_chassis_interfaces/action/MoveToMarker` | FSM navigation B,C,D,E,F,G,A | CODE-VERIFIED |
| `/competition/task` | `std_msgs/msg/String` JSON | Full box plus four-item task contract | CODE-VERIFIED |
| `/competition/control` | `std_msgs/msg/String` JSON | Pause/resume/stop/reset event contract | CODE-VERIFIED |

The new FSM binds each navigation action to an expected `ARRIVED_<point>` frame and
only advances after the action result is successful. This is the current arrival-frame
contract; a camera-image classifier for visual keyframe matching remains UNVERIFIED.

### Cameras and perception

| Interface | Type | Status |
| --- | --- | --- |
| `/{left,right}_camera/{left,right}_camera/color/image_raw` | `sensor_msgs/msg/Image` | CODE-VERIFIED |
| `/{left,right}_camera/{left,right}_camera/aligned_depth_to_color/image_raw` | `sensor_msgs/msg/Image` | CODE-VERIFIED |
| `/{left,right}_camera/{left,right}_camera/aligned_depth_to_color/camera_info` | `sensor_msgs/msg/CameraInfo` | CODE-VERIFIED |
| `/camera/camera/color/image_raw` | `sensor_msgs/msg/Image` | Configured generic/head fallback; role UNVERIFIED |
| `/qwen_vl/prompt` | `std_msgs/msg/String` | CODE-VERIFIED |
| `/qwen_vl/{left,right}/result` | `std_msgs/msg/String` JSON | CODE-VERIFIED |
| `/qwen_vl/{left,right}/annotated_image` | `sensor_msgs/msg/Image` | CODE-VERIFIED |
| `/yolov8/{left,right}/detections` | `yolov8_ros2/msg/Detection` | CODE-VERIFIED |
| `/yolov8/detect_control` | `yolov8_ros2/msg/DetectControl` | CODE-VERIFIED |
| `/yolov8/stream_control` | `yolov8_ros2/msg/StreamControl` | CODE-VERIFIED |

The integrated README records left/right RealSense serials `335222076738` and
`405622075108`. Live camera rates, serial ownership and the generic head-camera role
must still be verified on the robot.

### GraspNet, CuRobo and NVBlox

| Interface | Type | Status |
| --- | --- | --- |
| `/{left,right}/grasp/trigger` | `std_srvs/srv/Trigger` | CODE-VERIFIED |
| `/{left,right}/grasp/status` | `std_msgs/msg/String` | CODE-VERIFIED |
| `/{left,right}/grasp/candidates` | `std_msgs/msg/String` JSON | CODE-VERIFIED |
| `/{left,right}/grasp/pregrasp_pose` | `geometry_msgs/msg/PoseStamped` | CODE-VERIFIED |
| `/{left,right}/grasp/generated_target_pose` | `geometry_msgs/msg/PoseStamped` | CODE-VERIFIED |
| `/{left,right}/target_pose` | `geometry_msgs/msg/PoseStamped` | CODE-VERIFIED |
| `/{left,right}/curobo/status` | `std_msgs/msg/String` | CODE-VERIFIED |
| `/curobo_realman_planner/get_parameters` | `rcl_interfaces/srv/GetParameters` | CODE-VERIFIED |
| `/nvblox_node/get_esdf_and_gradient` | `nvblox_msgs/srv/EsdfAndGradients` | Supported by CuRobo; live use UNVERIFIED |

Current grasp defaults come from `supermarket_grasp_ros2/config/grasp.yaml`:
GraspNet source, CuRobo planner backend, `depth_scale=0.001`, synchronized-frame
tolerance `0.25 s`, Qwen ACCEPT filtering, and native execution disabled. The script
and conda paths still point to `/home/lh/Supermarket/grasp.py` and
`/home/lh/miniconda3/envs/grasp/bin/python`; deployment validity is UNVERIFIED.

### Robot drivers

| Interface | Type | Status |
| --- | --- | --- |
| `/{left,right}/rm_driver/movej_cmd` | `rm_ros_interfaces/msg/Movej` | CODE-VERIFIED |
| `/{left,right}/rm_driver/movej_result` | `std_msgs/msg/Bool` | CODE-VERIFIED |
| `/{left,right}/joint_states` | `sensor_msgs/msg/JointState` | CODE-VERIFIED |
| `/left/rm_driver/set_lift_height_cmd` | `rm_ros_interfaces/msg/Liftheight` | CODE-VERIFIED |
| `/left/rm_driver/set_lift_speed_cmd` | `rm_ros_interfaces/msg/Liftspeed` | CODE-VERIFIED |
| `/left/rm_driver/udp_lift_state` | `rm_ros_interfaces/msg/Udpliftstate` | Driver-supported; disabled in current driver YAML |
| `/{left,right}/omnipicker_gripper/open` | `std_srvs/srv/Trigger` | CODE-VERIFIED |
| `/{left,right}/omnipicker_gripper/close` | `std_srvs/srv/Trigger` | CODE-VERIFIED |

The integrated README records arm IPs `169.254.128.18` (left) and
`169.254.128.19` (right). Hardware connectivity was not tested.

## Console implementation status

| Feature | Status |
| --- | --- |
| Reference-image single-window layout | VERIFIED by offscreen screenshot |
| Task input and local validation | VERIFIED |
| Box and four-object progress | VERIFIED in mock |
| Full box/object state visualization | VERIFIED in mock |
| ROS worker isolation from `MainWindow` | VERIFIED by code and mock smoke test |
| Camera tabs and bbox overlay | VERIFIED in mock; live UNVERIFIED |
| Structured Qwen/YOLO result panel | VERIFIED in mock; live schema tolerant |
| Grasp candidates and selected metrics | VERIFIED in mock; live UNVERIFIED |
| Interactive 3D viewer | VTK on desktop, software fallback headless; VERIFIED in mock |
| NVBlox mesh/ESDF rendering | MISSING (health/service visibility only) |
| Full robot URDF rendering | MISSING (schematic arm only) |
| CuRobo trajectory message rendering | MISSING contract; mock trajectory only |
| Log tabs/search/error filter/export/cache | VERIFIED |
| Pause/resume/reset in real FSM | CODE-VERIFIED at `/competition/*` services |
| Physical emergency stop state/control | MISSING; console shows UNKNOWN and never claims task stop is E-stop |

## Integration boundary

The GUI does not start, stop or modify Qwen, YOLO, GraspNet, CuRobo, robot driver or
camera algorithms. `CompetitionRosBridge` is the only ROS boundary visible to the
widgets. Real mode publishes a versioned task JSON, then calls the existing navigation
start service. Full task consumption should be added to the state-machine package,
not to `MainWindow`.
