"""Competition states shown by the console and exercised by mock mode."""

TOP_STATES = (
    "SYSTEM_INIT", "WAIT_FOR_TASK", "VALIDATE_TASK", "EMPTY_BOX_TASK",
    "OBJECT_TASK_LOOP", "LOADED_BOX_TASK", "FINISHED", "SAFE_STOP", "ERROR",
)

BOX_STATES = (
    "EMPTY_BOX_TASK", "EMPTY_BOX_PHOTO_POSE", "EMPTY_BOX_NAVIGATE",
    "EMPTY_BOX_ARRIVAL_KEYFRAME", "EMPTY_BOX_LIFT_PICK", "EMPTY_BOX_PERCEPTION_LEFT",
    "EMPTY_BOX_PERCEPTION_RIGHT", "EMPTY_BOX_GRASPNET_LEFT", "EMPTY_BOX_GRASPNET_RIGHT",
    "EMPTY_BOX_GRASP_SELECTION", "EMPTY_BOX_CUROBO_PLAN_LEFT", "EMPTY_BOX_CUROBO_PLAN_RIGHT",
    "EMPTY_BOX_PLAN_BARRIER", "EMPTY_BOX_PREGRASP_BARRIER", "EMPTY_BOX_APPROACH_BARRIER",
    "EMPTY_BOX_GRASP_POSE_BARRIER", "EMPTY_BOX_CLOSE_BARRIER", "EMPTY_BOX_HOLD",
    "EMPTY_BOX_LIFT_TRANSPORT", "EMPTY_BOX_AGV_RETREAT", "EMPTY_BOX_NAVIGATE_G",
    "EMPTY_BOX_ARRIVAL_G", "EMPTY_BOX_LIFT_RELEASE", "EMPTY_BOX_OPEN_BARRIER",
    "EMPTY_BOX_PHOTO_COMPLETE", "EMPTY_BOX_COMPLETE", "DUAL_ARM_GRASP_BOX",
)

OBJECT_STATES = (
    "LOAD_TASK", "NAVIGATE_TO_SHELF", "NAVIGATE_TO_B", "NAVIGATE_TO_C", "NAVIGATE_TO_D",
    "NAVIGATE_TO_E", "NAVIGATE_TO_F", "NAVIGATE_TO_G", "ARRIVAL_KEYFRAME", "ARRIVAL_G",
    "ARM_TO_LIFT_SAFE",
    "LIFT_TO_PICK_HEIGHT",
    "LIFT_VERIFY",
    "ARM_TO_OBSERVE_POSE",
    "CAMERA_READY",
    "KEYFRAME_MATCH",
    "QWEN_YOLO_PERCEPTION",
    "NVBLOX_LOCAL_RECONSTRUCTION",
    "LEFT_RIGHT_GRASPNET",
    "GRASP_SELECTION",
    "CUROBO_PREGRASP_PLAN",
    "EXECUTE_PREGRASP",
    "CUROBO_APPROACH_PLAN",
    "EXECUTE_GRASP",
    "CLOSE_GRIPPER",
    "GRASP_VERIFY",
    "RETREAT",
    "ARM_TO_TRANSPORT_POSE",
    "LIFT_TO_POST_PICK_HEIGHT",
    "RETURN_TO_BOX",
    "LIFT_TO_PLACE_HEIGHT",
    "CUROBO_PLACE_PLAN",
    "PLACE_IN_BOX",
    "OPEN_GRIPPER",
    "RETREAT_FROM_BOX",
    "PLACE_VERIFY",
    "NEXT_OBJECT",
)

LOADED_BOX_STATES = (
    "LOADED_BOX_TASK", "LOADED_BOX_PHOTO_POSE", "LOADED_BOX_LIFT_SAFE",
    "LOADED_BOX_PERCEPTION_LEFT", "LOADED_BOX_PERCEPTION_RIGHT", "LOADED_BOX_GRASPNET_LEFT",
    "LOADED_BOX_GRASPNET_RIGHT", "LOADED_BOX_CUROBO_LEFT", "LOADED_BOX_CUROBO_RIGHT",
    "LOADED_BOX_PLAN_BARRIER", "LOADED_BOX_PREGRASP_BARRIER", "LOADED_BOX_APPROACH_BARRIER",
    "LOADED_BOX_CLOSE_BARRIER", "LOADED_BOX_VERIFY", "LOADED_BOX_NAVIGATE_A",
    "LOADED_BOX_ARRIVAL_A", "LOADED_BOX_NAVIGATE_K", "LOADED_BOX_ARRIVAL_K",
    "LOADED_BOX_LIFT_PLACE", "LOADED_BOX_PLACE_POSE",
    "LOADED_BOX_PLACE", "LOADED_BOX_OPEN_BARRIER", "LOADED_BOX_PLACE_COMPLETE",
)

STATE_LABELS = {
    "SYSTEM_INIT": "系统初始化",
    "INIT_BARRIER": "初始化就绪 barrier",
    "WAIT_FOR_TASK": "等待 Qt 任务",
    "VALIDATE_TASK": "校验任务配置",
    "EMPTY_BOX_TASK": "空箱搬运阶段",
    "OBJECT_TASK_LOOP": "商品任务循环",
    "LOADED_BOX_TASK": "满载箱搬运阶段",
    "FINISHED": "比赛流程完成",
    "SAFE_STOP": "安全停止",
    "ERROR": "错误",
    "IDLE": "等待任务",
    "LOAD_BOX_TASK": "载入箱体任务",
    "NAVIGATE_TO_BOX": "导航至箱体区",
    "LIFT_TO_BOX_HEIGHT": "升降至箱体高度",
    "BOX_PERCEPTION": "箱体感知",
    "BOX_GRASP_PLAN": "箱体抓取规划",
    "DUAL_ARM_GRASP_BOX": "双臂抓取箱体",
    "BOX_GRASP_VERIFY": "箱体抓取确认",
    "TRANSPORT_BOX": "运输箱体",
    "PLACE_BOX": "放置箱体",
    "BOX_PLACE_VERIFY": "箱体放置确认",
    "BOX_TASK_COMPLETE": "箱体任务完成",
    "LOADED_BOX_NAVIGATE_A": "导航至最终点 A",
    "LOADED_BOX_ARRIVAL_A": "已到达最终点 A",
    "LOADED_BOX_NAVIGATE_K": "导航至最终点 K",
    "LOADED_BOX_ARRIVAL_K": "已到达最终点 K",
    "LOAD_TASK": "载入商品任务",
    "NAVIGATE_TO_SHELF": "导航至货架",
    "NAVIGATION_VERIFY": "导航到达确认",
    "ARM_TO_LIFT_SAFE": "机械臂进入升降安全位",
    "LIFT_TO_PICK_HEIGHT": "升降至抓取高度",
    "LIFT_VERIFY": "升降高度确认",
    "ARM_TO_OBSERVE_POSE": "机械臂进入观察位",
    "CAMERA_READY": "相机就绪",
    "KEYFRAME_MATCH": "导航关键帧匹配",
    "QWEN_YOLO_PERCEPTION": "Qwen / YOLO 感知",
    "NVBLOX_LOCAL_RECONSTRUCTION": "NVBlox 局部重建",
    "LEFT_RIGHT_GRASPNET": "双臂 GraspNet 推理",
    "GRASP_SELECTION": "抓取候选选择",
    "CUROBO_PREGRASP_PLAN": "CuRobo 预抓取规划",
    "EXECUTE_PREGRASP": "执行预抓取",
    "CUROBO_APPROACH_PLAN": "CuRobo 接近规划",
    "EXECUTE_GRASP": "执行抓取",
    "CLOSE_GRIPPER": "闭合夹爪",
    "GRASP_VERIFY": "抓取确认",
    "RETREAT": "抓取后撤离",
    "ARM_TO_TRANSPORT_POSE": "机械臂进入运输位",
    "LIFT_TO_POST_PICK_HEIGHT": "升降至运输高度",
    "RETURN_TO_BOX": "返回箱体区",
    "LIFT_TO_PLACE_HEIGHT": "升降至放置高度",
    "CUROBO_PLACE_PLAN": "CuRobo 放置规划",
    "PLACE_IN_BOX": "放入箱体",
    "OPEN_GRIPPER": "打开夹爪",
    "RETREAT_FROM_BOX": "离开箱体",
    "PLACE_VERIFY": "放置确认",
    "NEXT_OBJECT": "切换下一商品",
    "COMPETITION_FINISHED": "比赛流程完成",
}


def state_label(state: str) -> str:
    if state.startswith("OBJECT_"):
        parts = state.split("_", 2)
        if len(parts) == 3:
            return f"商品 {parts[1]}：{state_label(parts[2])}"
    if state.startswith("LOADED_BOX_"):
        return "满载箱：" + STATE_LABELS.get(state, state.replace("LOADED_BOX_", ""))
    if state.startswith("NAVIGATING_TO_"):
        return f"导航至比赛点 {state.rsplit('_', 1)[-1]}"
    if state.startswith("TASK_AT_"):
        return f"比赛点 {state.rsplit('_', 1)[-1]} 任务占位"
    return STATE_LABELS.get(state, state or "未知")
