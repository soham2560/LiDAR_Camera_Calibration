import datetime
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown, SetEnvironmentVariable, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='namespace'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'record',
            default_value='False',
            description='Record in rosbag'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_rviz',
            default_value='False',
            description='Launch RVIZ on startup'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'enable_hardware',
            default_value='False',
            description='Enable hardware drivers'))
    declared_arguments.append(
        SetEnvironmentVariable(
            'RCUTILS_COLORIZED_OUTPUT', '1'))

    namespace = LaunchConfiguration('namespace')
    record = LaunchConfiguration('record')
    use_rviz = LaunchConfiguration('use_rviz')
    enable_hardware = LaunchConfiguration('enable_hardware')

    package_path = get_package_share_directory('lidar_camera_calibration')
    rs_package_path = get_package_share_directory('realsense2_camera')

    get_current_timestamp = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    log_full_path = os.path.join('/ros2_ws/src/records/', get_current_timestamp)
    rosbag_full_path = os.path.join(log_full_path, 'rosbag')

    config_dir = os.path.join(package_path, 'config')
    calibrate_config_path = os.path.join(config_dir, 'calibration.yaml')
    zhangs_config_path = os.path.join(config_dir, 'zhangs.yaml')
    realsense_config_path = os.path.join(config_dir, 'realsense.yaml')

    xacro_path = PathJoinSubstitution(
        [package_path, 'urdf', 'robot.urdf.xacro']
    )

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        namespace=namespace,
        parameters=[
            {
                'frame_prefix': [namespace, '/'],
                'robot_description': ParameterValue(
                    Command(['xacro ', xacro_path, ' ',
                            'NAMESPACE:=', namespace, ' ']), value_type=str),
            }
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        namespace=namespace,
        arguments=['-d', package_path + '/rviz/robot.rviz'],
        on_exit=Shutdown(),
        condition=IfCondition(use_rviz),
    )

    rosbag_recorder_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [package_path, '/launch/rosbag_recorder.launch.py']
                ),
                launch_arguments={
                    'rosbag_storage_dir': rosbag_full_path,
                }.items(),
                condition=IfCondition(record),
            )
        ]
    )

    velodyne_hw_if = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('lidar_camera_calibration'),
                              'launch', 'velodyne_hw_if.launch.py')]),
            condition=IfCondition(enable_hardware))

    camera_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
                [os.path.join(rs_package_path,
                              'examples','launch_params_from_file', 'rs_launch_get_params_from_yaml.py')]),
        launch_arguments={
            'camera_name': "camera",
            'camera_namespace': "camera",
            'config_file': realsense_config_path
        }.items(),
        condition=IfCondition(enable_hardware)
    )

    sync_sensors = Node(
        package='lidar_camera_calibration',
        executable='sync_sensors',
        name='sync_sensors',
        output='screen',
        parameters=[
            {'topic_image': '/camera/camera/color/image_raw'},
            {'topic_pointcloud': '/velodyne_points'},
            {'queue_size': 30},
            {'use_approximate_sync': True}
        ],
        condition=IfCondition(enable_hardware)
    )

    zhangs_node = Node(
        package='lidar_camera_calibration',
        executable='zhangs',
        name='zhang_calibration',
        output='screen',
        parameters=[zhangs_config_path]
    )
    calibration_node = RegisterEventHandler(
        OnProcessExit(
            target_action=zhangs_node,
            on_exit=[
                Node(
                    package='lidar_camera_calibration',
                    executable='calibration',
                    name='calibration',
                    output='screen',
                    parameters=[calibrate_config_path]
                )
            ]
        )
    )

    nodes = [
        robot_state_pub_node,
        rviz_node,
        rosbag_recorder_launch,
        calibration_node,
        zhangs_node,
        sync_sensors,
        velodyne_hw_if,
        camera_driver
    ]

    return LaunchDescription(declared_arguments + nodes)