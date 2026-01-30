from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'world': os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                'worlds',
                'turtlebot3_world.world'
            )
        }.items()
    )

    spawn_tb3 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'burger',
            '-file',
            os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                'models',
                'turtlebot3_burger',
                'model.sdf'
            ),
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.01'
        ],
        output='screen'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'publish_frequency': 50.0,
            'use_sim_time': True,
            'publish_default_tf': False
        }],
        remappings=[('/joint_states', '/burger/joint_states')]
    )

    return LaunchDescription([
        gazebo_launch,
        spawn_tb3,
        robot_state_publisher_node
    ])
