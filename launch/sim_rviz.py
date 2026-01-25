from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 1. Setup Paths
    package_dir = get_package_share_directory("simulation")
    urdf_path = os.path.join(package_dir, 'udrf', 'sjtu_drone.urdf')

    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        # Node: Robot State Publisher (Visualizes the drone)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}]
        ),

        # Node: Static TF for LIDAR 1 (Front/Sides)
        # Arguments: x y z yaw pitch roll parent_frame child_frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_lidar_front',
            arguments=['0.1', '0', '0', '0', '0',
                       '0', 'base_link', 'lidar_front_frame']
        ),

        # Node: Static TF for LIDAR 2 (Above/Below/Sides)
        # We might rotate this one 90 degrees (1.5708 radians) on the pitch or roll axis
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_lidar_vertical',
            arguments=[
                '--x', '0.15',
                '--y', '0',
                '--z', '0.05',
                '--yaw', '0',
                '--pitch', '-1.5708',
                '--roll', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'lidar_vertical_frame'
            ]
        ),

        # Node: Launch RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', './rviz_sim.rviz']
        ),
        # Launch the simulation interfaces
        Node(
            package='simulation',
            executable='control',
            name='sim_ctrl',
            ),
        Node(
            package='simulation',
            executable='lidarstreams',
            name='sim_lidar',
            ),
        # Launch the driver board interface
        Node(
            package='control_board',
            executable='driver',
            name='cb_interface',
            ),
    ])
