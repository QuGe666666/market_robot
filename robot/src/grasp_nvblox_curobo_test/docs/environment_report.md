# Environment report

Inspection date: 2026-08-20 (Asia/Shanghai).

This file records observed local state. It is not a list of assumed interfaces.

| Item | Observed value |
| --- | --- |
| OS | Ubuntu 22.04, aarch64, Jetson Linux 5.15.148-tegra |
| ROS 2 | Humble |
| Isaac ROS | release-3.1 workspaces; `isaac_ros_nvblox` package version 3.1.0, commit `bbc0972` |
| nvblox | Source and build present under `/home/lh/isaac_ros_ws`; ESDF service type is `nvblox_msgs/srv/EsdfAndGradients` |
| cuRobo | Docker base image `curobo-jetson:v0.7.8-r36.4`; Python distribution reports 0.0.0 because it was built from source |
| cuMotion | No cuMotion ROS package or Python module found |
| CUDA | 12.6.11 toolkit; PyTorch runtime compiled for CUDA 12.6 |
| PyTorch | `2.5.0a0+872d972e41.nv24.08` in `lerobot-realman-orin:stage1` |
| Python | ROS/container Python 3.10.12; interactive user shell currently prefers Conda Python 3.13.5 |
| GraspNet | `/home/lh/Supermarket/graspnet-baseline`; checkpoint `checkpoint-rs.tar` loaded successfully on CUDA |
| Robot | Dual RealMan RM65, six joints per arm |
| Robot model | `/home/lh/robot/src/curobo_realman_test/config/rm65.urdf` and `rm65.yml` |
| Collision model | 12 configured link spheres over Link1 through Link6; self-collision checks are disabled in the existing validated planner config |
| RGB-D | Three Intel RealSense D435 devices detected; right wrist serial is configured as `405622075108` |

## Live ROS graph observed before implementation

Nodes:

```text
/curobo_realman_planner
/left/rm_driver
/right/rm_driver
/right_wrist_camera_tf
```

Relevant topics:

```text
/left/joint_states                         sensor_msgs/msg/JointState
/right/joint_states                        sensor_msgs/msg/JointState
/left/target_pose                          geometry_msgs/msg/PoseStamped
/right/target_pose                         geometry_msgs/msg/PoseStamped
/left/curobo/trajectory                    trajectory_msgs/msg/JointTrajectory
/right/curobo/trajectory                   trajectory_msgs/msg/JointTrajectory
/left/curobo/status                        std_msgs/msg/String
/right/curobo/status                       std_msgs/msg/String
/tf                                       tf2_msgs/msg/TFMessage
/tf_static                                tf2_msgs/msg/TFMessage
```

The camera and nvblox processes were not running during the initial graph snapshot, so no camera,
mesh, or ESDF service was advertised at that moment. Hardware enumeration did find the cameras.

Observed wrist transform once the broadcaster produced data:

```text
right_base -> right_camera_color_optical_frame
translation approximately [-0.405, 0.036, 0.090] m
```

The test planning frame is therefore `right_base`, not the generic `base_link` from the task
template. Internally the reused CuRobo URDF root is `driver_base`; validation launch publishes an
identity `right_base -> driver_base` transform for RViz robot display.

## Existing modules reused

- RM65 URDF, joint limits, collision spheres, and Docker runner pattern from
  `/home/lh/robot/src/curobo_realman_test`.
- Exact nvblox 3.1 ESDF service and `x/y/z` Float32MultiArray layout from local source.
- Existing aligned RealSense topics:
  `/right_camera/right_camera/aligned_depth_to_color/image_raw` and matching CameraInfo.
- Existing calibrated dynamic TF broadcaster for
  `right_base -> right_camera_color_optical_frame`.

The formal modules above were not modified.

## GraspNet alignment finding and validation adapter

The existing `/grounded_sam2/right/grasps` bridge publishes JSON as `std_msgs/String`, does not
carry the source image timestamp, and calculates its base-frame result using the arm pose available
after inference. It cannot prove `T_base_camera(t_image)` and is deliberately not used by this
validation package.

`timestamped_graspnet_node.py` is the corrected boundary. It caches aligned depth and CameraInfo by
the complete `(sec, nanosec)` key, consumes the Grounded SAM2 mask carrying the original RGB stamp,
and publishes `PoseStamped` in `right_camera_color_optical_frame` with that unchanged stamp. A mask
shifted by one nanosecond was rejected with `TF_ERROR`; the exact matching camera sample
`1787221590.108819580` produced `GRASPNET_VALID` after real CUDA inference with a diagnostic mask.
The full Grounded SAM2 prompt path was then exercised with the visible `cup`: six GraspNet
candidates were produced and both `/grasp_pose_camera` and `/grasp_pose_base` retained the exact
stamp `1787223236.704305908`. The original legacy bridge remains untouched.
