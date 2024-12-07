import datetime
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown, SetEnvironmentVariable, IncludeLaunchDescription, TimerAction, ExecuteProcess
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

    data_base_path = "/ros2_ws/src/lidar_camera_calibration/data/rosbag_data"
    rosbag_path = os.path.join(data_base_path, "1280rosbags")
    rosbag_extract_path = os.path.join(data_base_path, "rosbag_extract")



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

    # Preprocess command
    preprocess_node = ExecuteProcess(
        cmd=['ros2', 'run', 'lidar_camera_calibration', 'preprocess', 
             rosbag_path, 
             rosbag_extract_path, 
             '-ad'],
        name='preprocess',
        output='screen'
    )

    # Find matches with SuperGlue command (triggered after preprocess finishes)
    find_matches_node = ExecuteProcess(
        cmd=['ros2', 'run', 'lidar_camera_calibration', 'find_matches_superglue.py', 
             rosbag_extract_path],
        name='find_matches',
        output='screen'
    )

    # Find correspondences command (triggered after find_matches finishes)
    find_correspondences_node = ExecuteProcess(
        cmd=['ros2', 'run', 'lidar_camera_calibration', 'find_correspondences', 
             rosbag_extract_path],
        name='find_correspondences',
        output='screen'
    )

    find_matches_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=preprocess_node,
            on_exit=[find_matches_node]
        )
    )

    find_correspondences_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=find_matches_node,
            on_exit=[find_correspondences_node]
        )
    )

    zhangs_node = Node(
        package='lidar_camera_calibration',
        executable='zhangs',
        name='zhang_calibration',
        output='screen',
        parameters=[zhangs_config_path]
    )

    zhangs_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=find_correspondences_node,
            on_exit=[zhangs_node]
        )
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
        # sync_sensors,
        preprocess_node , # preprocess_handler,
        find_matches_handler,
        find_correspondences_handler,
        zhangs_handler,
        calibration_node,
        velodyne_hw_if,
        camera_driver
    ]

    return LaunchDescription(declared_arguments + nodes)