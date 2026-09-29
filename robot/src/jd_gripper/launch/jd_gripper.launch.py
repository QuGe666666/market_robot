from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "arm_ip",
            default_value="192.168.1.18",
            description="IP address of the robot arm controller",
        ),
        DeclareLaunchArgument(
            "arm_port",
            default_value="8080",
            description="Port of the robot arm controller",
        ),
        DeclareLaunchArgument("serial_device", default_value="/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG01DLWP-if00-port0",
                              description="Local USB-RS485 device connected to OmniPicker"),
        DeclareLaunchArgument(
            "gripper_device",
            default_value="1", description="OmniPicker node ID",
        ),
        DeclareLaunchArgument("baudrate", default_value="115200", description="OmniPicker serial baudrate"),
        DeclareLaunchArgument(
            "default_speed",
            default_value="255",
            description="Default gripper speed (0-255)",
        ),
        DeclareLaunchArgument(
            "default_force",
            default_value="255",
            description="Default gripper force (0-255)",
        ),
        DeclareLaunchArgument("tool_voltage", default_value="3", description="RM tool voltage: 0=off, 2=12V, 3=24V"),
        DeclareLaunchArgument("enable_tool_power", default_value="true", description="Configure RM tool power"),
        DeclareLaunchArgument(
            "auto_init",
            default_value="true",
            description="Automatically initialize gripper on startup",
        ),
        DeclareLaunchArgument(
            "publish_rate",
            default_value="10.0",
            description="State publishing rate in Hz",
        ),
        DeclareLaunchArgument(
            "use_config_file",
            default_value="false",
            description="Whether to use config file for parameters",
        ),
        DeclareLaunchArgument(
            "config_file",
            default_value="gripper_params.yaml",
            description="Name of the config file",
        ),
    ]

    jd_gripper_node = Node(
        package="jd_gripper",
        executable="jd_gripper_node",
        name="jd_gripper_node",
        output="screen",
        parameters=[
            {"arm_ip": LaunchConfiguration("arm_ip")},
            {"arm_port": LaunchConfiguration("arm_port")},
            {"serial_device": LaunchConfiguration("serial_device")},
            {"gripper_device": LaunchConfiguration("gripper_device")},
            {"baudrate": LaunchConfiguration("baudrate")},
            {"default_speed": LaunchConfiguration("default_speed")},
            {"default_force": LaunchConfiguration("default_force")},
            {"tool_voltage": LaunchConfiguration("tool_voltage")},
            {"enable_tool_power": LaunchConfiguration("enable_tool_power")},
            {"auto_init": LaunchConfiguration("auto_init")},
            {"publish_rate": LaunchConfiguration("publish_rate")},
        ],
        remappings=[
            ("~/cmd", "/jd_gripper/cmd"),
            ("~/state", "/jd_gripper/state"),
            ("~/is_holding", "/jd_gripper/is_holding"),
            ("~/position", "/jd_gripper/position"),
            ("~/init", "/jd_gripper/init"),
            ("~/open", "/jd_gripper/open"),
            ("~/close", "/jd_gripper/close"),
            ("~/grasp", "/jd_gripper/grasp"),
        ],
    )

    return LaunchDescription(declared_arguments + [jd_gripper_node])
