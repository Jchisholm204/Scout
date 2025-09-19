from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ''use_sim_time'' is used to have ros2 use /clock topic for the time source
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # urdf = FileContent(
    #         PathJoinSubstitution([FindPackageShare('Scout'), 'urdf', 'sjtu_drone.urdf']))
# Get the workspace directory (relative from this launch file)
    ws_dir = os.path.dirname(os.path.realpath(__file__))  # directory of this launch file
    urdf_path = os.path.join(ws_dir, '..', 'udrf', 'sjtu_drone.urdf')  # adjust depth as needed
    urdf_path = os.path.abspath(urdf_path)  # ensure absolute path

    # Read the URDF file manually instead of using FileContent
    with open(urdf_path, 'r') as urdf_file:
        urdf = urdf_file.read()

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
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': urdf}],
            arguments=[urdf]
        ),
        Node(
            package='interfaces',
            executable='inav_bridge',
            name='inav_bridge',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            # arguments=['-d', os.path.join(ws_dir, '..', '..', 'rviz', 'drone_config.rviz')]
        ),
    ])
