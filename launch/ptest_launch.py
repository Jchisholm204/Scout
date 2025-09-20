from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    package_dir = get_package_share_directory("interfaces")
    urdf = f"{package_dir}/udrf/sjtu_drone.urdf"
    with open(urdf, 'r') as udrf_file:
        urdf = udrf_file.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        Node(
            package='interfaces',
            executable='rplidar_bridge',
            name='rplidar_bridge',
            parameters=[{
                'lidar_path': '/dev/ttyUSB0',
                'lidar_mode': 0
            }],
        ),
        Node(
            package='interfaces',
            executable='inav_bridge',
            name='inav_bridge',
            parameters=[{
                'fc_path': '/dev/ttyACM0',
                'fc_baud': 921600,
                'fall_speed': 0,
            }],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                         'robot_description': urdf}],
            arguments=[urdf]),
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='screen',
        #     # arguments=['-d', os.path.join(ws_dir, '..', '..', 'rviz', 'drone_config.rviz')]
        # ),
    ])
