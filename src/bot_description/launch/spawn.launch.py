import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    package_name = 'bot_description'
    
    # Include robot_state_publisher launch file with sim time enabled
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package_name), 'launch', 'rviz.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Specify the hospital world file from your package
    hospital_world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'hospital.world'
    )

    # Include Gazebo launch file; pass the hospital world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': hospital_world, 'gui': 'true'}.items()
    )

    # Delay the spawn entity node to ensure Gazebo's plugins have loaded
    spawn_entity = TimerAction(
        period=10.0,  # Increase delay if necessary
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
                output='screen'
            )
        ]
    )

    # Launch teleop_twist_keyboard in a new terminal window (GUI version)
    teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e'
    )

    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        teleop,
    ])
