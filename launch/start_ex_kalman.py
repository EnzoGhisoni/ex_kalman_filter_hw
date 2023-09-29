import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare a launch argument for the RViz configuration file path
    rviz_config_file = DeclareLaunchArgument(
        'Displays',
        default_value='/home/enzo/Documents/Codes/ros2_ws/src/ex_kalman_filter_hw/launch/start_ex_kalman.py',
        description='Path to the RViz configuration file'
    )

    return LaunchDescription([
        # Add the RViz configuration file argument to the launch description
        rviz_config_file,
        # Launch the node
        Node(
            package='ex_kalman_filter_hw',
            executable='ex_kalman_node',
            name='ex_kalman_filter',
            output='screen',
            parameters=[],
        ),

        Node(
            package='ex_kalman_filter_hw',
            executable='republish_bag_node',
            name='republish_bag',
            output='screen',
            parameters=[],
        ),

        # Launch RViz2 with the specified configuration file
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config_file')],
            remappings=[('/tf', '/tf')],
            parameters=[],
        ),
    ])
