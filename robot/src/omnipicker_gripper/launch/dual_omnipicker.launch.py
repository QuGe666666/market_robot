"""Launch two OmniPickers through the two Realman末端 Modbus ports."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    arguments = [
        DeclareLaunchArgument("left_arm_ip", default_value="169.254.128.18"),
        DeclareLaunchArgument("right_arm_ip", default_value="169.254.128.19"),
        DeclareLaunchArgument("arm_port", default_value="8080"),
        DeclareLaunchArgument("left_gripper_id", default_value="2",
                              description="Left OmniPicker ID (SN 0000336E3335)"),
        DeclareLaunchArgument("right_gripper_id", default_value="3",
                              description="Right OmniPicker ID (SN 0000396B5245)"),
        DeclareLaunchArgument("modbus_port", default_value="1"),
        DeclareLaunchArgument("baudrate", default_value="115200"),
        DeclareLaunchArgument("default_speed", default_value="255"),
        DeclareLaunchArgument("default_force", default_value="255"),
        DeclareLaunchArgument("tool_voltage", default_value="3"),
        DeclareLaunchArgument("enable_tool_power", default_value="true"),
        DeclareLaunchArgument("auto_init", default_value="true"),
        DeclareLaunchArgument("publish_rate", default_value="10.0"),
        DeclareLaunchArgument("status_poll_rate", default_value="5.0"),
        DeclareLaunchArgument(
            "holding_status_code",
            default_value="-1",
            description="OmniPicker register-21 value meaning holding; -1 leaves it unclassified",
        ),
    ]
    node = Node(
        package="omnipicker_gripper",
        executable="omnipicker_modbus_node",
        name="omnipicker_modbus_node",
        output="screen",
        parameters=[{
            name: LaunchConfiguration(name)
            for name in (
                "left_arm_ip", "right_arm_ip", "arm_port", "modbus_port",
                "left_gripper_id", "right_gripper_id", "baudrate",
                "default_speed", "default_force", "tool_voltage",
                "enable_tool_power", "auto_init", "publish_rate", "status_poll_rate",
                "holding_status_code",
            )
        }],
        remappings=[
            (f"~/left/{endpoint}", f"/left/omnipicker_gripper/{endpoint}")
            for endpoint in ("cmd", "position", "position_valid", "status", "is_holding", "init", "open", "close", "grasp")
        ] + [
            (f"~/right/{endpoint}", f"/right/omnipicker_gripper/{endpoint}")
            for endpoint in ("cmd", "position", "position_valid", "status", "is_holding", "init", "open", "close", "grasp")
        ],
    )
    return LaunchDescription(arguments + [node])
