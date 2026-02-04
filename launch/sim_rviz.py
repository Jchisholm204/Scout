from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node, LifecycleNode
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 1. Setup Paths
    package_dir = get_package_share_directory("simulation")
    urdf_path = os.path.join(package_dir, 'udrf', 'sjtu_drone.urdf')

    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    # 1. Define the node
    seg_node = LifecycleNode(
        package='laser_segmentation',
        executable='laser_segmentation',
        name='lidar_segmentation',
        namespace='',
        output='screen',
        # parameters=[{
        #     'target_frame': 'lidar_front_frame',  # Or 'base_link'
        #     'segmentation_type': 'jump_distance_merge',
        #     'min_points_segment': 5,
        #     'max_distance_jump': 0.1,
        # }],
        parameters=[{
            'scan_topic': '/sim/ls_front',
            'target_frame': 'base_link',
            'segmentation_type': 'jump_distance_merge',

            # AGGRESSIVE MERGING (The "Error" Tolerance)
            'distance_threshold': 0.4,       # Large gap tolerance between points
            'max_distance_jump': 0.4,        # Merge segments even if they are far apart

            # GENERIC FILTERING
            'min_points_segment': 8,         # Accept even very "sparse" walls
            'max_points_segment': 500,       # Allow long continuous hallway walls
            'min_segment_width': 0.2,        # Don't throw away small wall chunks
            'max_segment_width': 15.0,       # Capture the whole side of a long hall

            # RANGE (Generic Hallway)
            'min_avg_distance_from_sensor': 0.1,
            'max_avg_distance_from_sensor': 10.0,
        }],
        remappings=[('/scan', '/cb/ls_front')]
    )

    # 2. Simple Command Line "Activators"
    # We wait 2 seconds after the node starts to trigger the states
    configure_cmd = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set',
                     '/lidar_segmentation', 'configure'],
                output='screen'
            )
        ]
    )

    activate_cmd = TimerAction(
        period=4.0,  # Wait a bit longer to ensure configuration finished
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set',
                     '/lidar_segmentation', 'activate'],
                output='screen'
            )
        ]
    )

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
        Node(
            package='simulation',
            executable='telemetry',
            name='sim_telemetry',
            parameters=[
                {"pub_rate": 50}
            ]
        ),
        # Launch the driver board interface
        Node(
            package='control_board',
            executable='driver',
            name='cb_interface',
        ),
        # Launch the LiDAR Segmentation Node
        # Node(
        #     package='laser_segmentation',
        #     executable='laser_segmentation',
        #     name='lidar_segmentation',
        #     parameters=[{
        #         'target_frame': 'lidar_front_frame',
        #         'segmentation_type': 'jump_distance_merge',  # Common algorithm
        #         'min_points': 5,
        #         'max_distance_jump': 0.1,
        #     }],
        #     remappings=[
        #         # Map 'scan' to your specific lidar topic name if it's different
        #         ('/scan', '/cb/ls_front'),
        #     ],
        #     output='screen'
        # ),
        seg_node,
        configure_cmd,
        activate_cmd,
    ])
