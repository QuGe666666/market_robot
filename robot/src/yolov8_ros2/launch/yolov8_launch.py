from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'model_path',
            default_value="/home/lh/robot/src/yolov8_ros2/yolov8_ros2/best.pt",
            description='YOLOv8 model path. Relative paths are resolved from cwd/package root/history model dir.'
        ),
        DeclareLaunchArgument('arm', default_value='right', description='Arm namespace for detections.'),
        DeclareLaunchArgument('infer_node_name', default_value='yolov8_infer_node'),
        DeclareLaunchArgument('detect_control_node_name', default_value='detect_control_node'),
        DeclareLaunchArgument('stream_control_node_name', default_value='stream_control_node'),
        DeclareLaunchArgument('label_aliases', default_value='{"超市":["0","chaoshi"],"奥利奥":["1","ao"],"阿萨姆奶茶":["2","asamu"],"加多宝":["3","bao"],"彩虹糖":["4","cai"],"果粒橙":["5","chengzi"],"脆升升":["6","cui"],"焦糖瓜子":["7","guazi"],"果粒爽":["8","guo"],"雀巢咖啡":["9","kafei"],"百事可乐":["10","kele"],"茉莉茶":["11","moli"],"好丽友派":["12","pai"],"薯片":["13","shu"],"哇哈哈":["14","wahaha"],"雪碧":["15","xuebi"],"益达":["16","yida"]}', description='JSON mapping from Chinese labels to YOLO ids/names.'),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/camera/camera/color/image_raw',
            description='Input color image topic for YOLO inference.'
        ),
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/camera/camera/aligned_depth_to_color/image_raw',
            description='Optional aligned depth image topic used to fill Detection.depth.'
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/camera/camera/aligned_depth_to_color/camera_info',
            description='CameraInfo topic kept for diagnostics and compatibility.'
        ),
        DeclareLaunchArgument(
            'use_depth',
            default_value='true',
            description='Use depth topic to compute object depth.'
        ),
        DeclareLaunchArgument(
            'enable_inference_on_start',
            default_value='false',
            description='Whether inference is enabled immediately after node startup.'
        ),
        DeclareLaunchArgument(
            'show_image',
            default_value='false',
            description='Show annotated inference frames in a local OpenCV window.'
        ),
        DeclareLaunchArgument(
            'warmup_on_start',
            default_value='true',
            description='Run one dummy inference during startup to keep the model framework initialized.'
        ),
        DeclareLaunchArgument(
            'device',
            default_value='',
            description='Ultralytics device argument, for example cpu, 0, cuda:0. Empty means auto.'
        ),
        DeclareLaunchArgument(
            'confidence_thresh',
            default_value='0.5',
            description='Default confidence threshold before /yolov8/detect_control overrides it.'
        ),
        DeclareLaunchArgument(
            'target_labels',
            default_value='',
            description='Comma-separated default class ids before /yolov8/detect_control overrides them.'
        ),
        DeclareLaunchArgument(
            'class_name_aliases',
            default_value='{"0":"超市","chaoshi":"超市","1":"奥利奥","ao":"奥利奥","2":"阿萨姆奶茶","asamu":"阿萨姆奶茶","3":"加多宝","bao":"加多宝","4":"彩虹糖","cai":"彩虹糖","5":"果粒橙","chengzi":"果粒橙","6":"脆升升","cui":"脆升升","7":"焦糖瓜子","guazi":"焦糖瓜子","8":"果粒爽","guo":"果粒爽","9":"雀巢咖啡","kafei":"雀巢咖啡","10":"百事可乐","kele":"百事可乐","11":"茉莉茶","moli":"茉莉茶","12":"好丽友派","pai":"好丽友派","13":"薯片","shu":"薯片","14":"哇哈哈","wahaha":"哇哈哈","15":"雪碧","xuebi":"雪碧","16":"益达","yida":"益达"}',
            description='JSON object used to show Chinese class names on annotated images. Keys may be model class names or class ids.'
        ),
        DeclareLaunchArgument(
            'max_inference_fps',
            default_value='2.0',
            description='Maximum inference FPS. Set <=0 to infer every received image.'
        ),
        DeclareLaunchArgument(
            'max_no_detection_attempts',
            default_value='2',
            description='Stop after this many processed frames without a target.',
        ),
        DeclareLaunchArgument(
            'detection_window_s',
            default_value='2.0',
            description='Collect detections for this many seconds before publishing the best candidate.',
        ),
        DeclareLaunchArgument(
            'log_dir',
            default_value='',
            description='File log directory. Empty means yolov8_ros2/logs under the package root.'
        ),
        DeclareLaunchArgument(
            'topic_diagnostic_interval_sec',
            default_value='5.0',
            description='Interval for logging ROS topic publisher/subscriber diagnostics. Set <=0 to disable.'
        ),
        DeclareLaunchArgument(
            'no_image_warn_after_sec',
            default_value='3.0',
            description='Warn when inference is enabled but no input image has been received after this many seconds.'
        ),
        DeclareLaunchArgument(
            'publish_annotated_image',
            default_value='true',
            description='Publish filtered 2D annotated image for RViz Image panel.'
        ),
        DeclareLaunchArgument(
            'annotated_image_topic',
            default_value='/yolov8/annotated_image',
            description='Output topic for the 2D annotated RGB image.'
        ),
        DeclareLaunchArgument(
            'start_stream_control_node',
            default_value='false',
            description='Start helper StreamControl publisher for manual tests.'
        ),
        Node(
            package='yolov8_ros2',
            executable='yolov8_infer_node',
            name=LaunchConfiguration('infer_node_name'),
            output='screen',
            prefix='/home/lh/miniconda3/envs/robot/bin/python',
            parameters=[{
                'model_path': LaunchConfiguration('model_path'),
                'arm': LaunchConfiguration('arm'),
                'label_aliases': ParameterValue(LaunchConfiguration('label_aliases'), value_type=str),
                'image_topic': LaunchConfiguration('image_topic'),
                'depth_topic': LaunchConfiguration('depth_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'use_depth': ParameterValue(LaunchConfiguration('use_depth'), value_type=bool),
                'enable_inference_on_start': ParameterValue(LaunchConfiguration('enable_inference_on_start'), value_type=bool),
                'show_image': ParameterValue(LaunchConfiguration('show_image'), value_type=bool),
                'warmup_on_start': ParameterValue(LaunchConfiguration('warmup_on_start'), value_type=bool),
                'device': LaunchConfiguration('device'),
                'confidence_thresh': ParameterValue(LaunchConfiguration('confidence_thresh'), value_type=float),
                'target_labels': LaunchConfiguration('target_labels'),
                'class_name_aliases': ParameterValue(LaunchConfiguration('class_name_aliases'), value_type=str),
                'max_inference_fps': ParameterValue(LaunchConfiguration('max_inference_fps'), value_type=float),
                'max_no_detection_attempts': ParameterValue(LaunchConfiguration('max_no_detection_attempts'), value_type=int),
                'detection_window_s': ParameterValue(LaunchConfiguration('detection_window_s'), value_type=float),
                'log_dir': LaunchConfiguration('log_dir'),
                'topic_diagnostic_interval_sec': ParameterValue(LaunchConfiguration('topic_diagnostic_interval_sec'), value_type=float),
                'no_image_warn_after_sec': ParameterValue(LaunchConfiguration('no_image_warn_after_sec'), value_type=float),
                'publish_annotated_image': ParameterValue(LaunchConfiguration('publish_annotated_image'), value_type=bool),
                'annotated_image_topic': LaunchConfiguration('annotated_image_topic'),
            }]
        ),
        Node(
            package='yolov8_ros2',
            executable='stream_control_node',
            name=LaunchConfiguration('stream_control_node_name'),
            output='screen',
            prefix='/home/lh/miniconda3/envs/robot/bin/python',
            condition=IfCondition(LaunchConfiguration('start_stream_control_node')),
        ),
        Node(
            package='yolov8_ros2',
            executable='detect_control_node',
            name=LaunchConfiguration('detect_control_node_name'),
            output='screen',
            prefix='/home/lh/miniconda3/envs/robot/bin/python',
        ),
    ])
