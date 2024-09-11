from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('local_planner'),
        'config',
        'local_planner.yaml'
    )
    
    event_type = DeclareLaunchArgument(
        'event_type',
        default_value="",
        description='Select the mission to execute'
    )

    local_planner_node = Node(
        name='local_planner_node',
        package='local_planner',
        executable='local_planner_node',
        parameters=[config,  {'event_type': LaunchConfiguration('event_type')}],
        output="screen"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='Use simulation (Gazebo) clock if true'
            ),
            event_type,
            local_planner_node
        ]
    )