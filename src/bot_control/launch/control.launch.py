import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'bot_description'
    
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package_name), 'launch', 'rviz.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    world_file = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'obstacles.world'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file, 'gui': 'true'}.items()
    )

    spawn_entity = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-topic', 'robot_description',
                    '-entity', 'my_bot',
                    '-x', '0',
                    '-y', '0',
                    '-z', '0'
                ],
                output='screen'
            )
        ]
    )

    teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e'
    )

    # ✅ Add your laser filtering node
    laser_filter_node = Node(
        package='bot_control',
        executable='reading_laser',
        name='reading_laser',
        output='screen'
    )

    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        teleop,
        laser_filter_node  # 👈 this line runs your laser filtering node
    ])
