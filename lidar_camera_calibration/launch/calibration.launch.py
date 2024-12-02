import datetime
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown, SetEnvironmentVariable, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, AndSubstitution, NotSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Launch in simulation mode.'))
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
        SetEnvironmentVariable(
            'RCUTILS_COLORIZED_OUTPUT', '1'))

    # Initialize Arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    record = LaunchConfiguration('record')
    use_rviz = LaunchConfiguration('use_rviz')

    # Package Path
    package_path = get_package_share_directory('lidar_camera_calibration')
    rs_package_path = get_package_share_directory('realsense2_camera')

    # set log output path
    get_current_timestamp = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    log_full_path = os.path.join('/ros2_ws/src/records/', get_current_timestamp)
    rosbag_full_path = os.path.join(log_full_path, 'rosbag')

    # Get URDF via xacro
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
                'use_sim_time': use_sim_time,
                'frame_prefix': [namespace, '/'],
                'robot_description': ParameterValue(
                    Command(['xacro ', xacro_path, ' ',
                            'USE_WITH_SIM:=', use_sim_time, ' ',
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
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', package_path + '/rviz/robot.rviz'],
        on_exit=Shutdown(),
        condition=IfCondition(use_rviz),
    )

    rosbag_recorder_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [package_path, '/launch/rosbag_recorder.launch.py']
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rosbag_storage_dir': rosbag_full_path,
        }.items(),
        condition=IfCondition(record),
    )

    velodyne_hw_if = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('lidar_camera_calibration'),
                              'launch', 'velodyne_hw_if.launch.py')]),
            condition=UnlessCondition(use_sim_time))

    camera_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
                [os.path.join(rs_package_path,
                              'launch/' 'rs_launch.py')]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
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
        ]
    )

    nodes = [
        robot_state_pub_node,
        rviz_node,
        rosbag_recorder_launch,
        sync_sensors,
        velodyne_hw_if,
        camera_driver
    ]

    return LaunchDescription(declared_arguments + nodes)
