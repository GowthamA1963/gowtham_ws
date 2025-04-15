import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'bot_description'
    
    # Include the robot_state_publisher launch file (which also launches RViz)
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package_name), 'launch', 'rviz.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Specify your world file—in this example, we're using 'my_world.sdf'
    # If you changed your world file name to obstacles.world or another name, update it accordingly.
    world_file = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'obstacles.world'
    )

    # Launch Gazebo with your world file and enable the GUI
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file, 'gui': 'true'}.items()
    )

    # Delay spawn of the robot to allow Gazebo to fully load the world and plugins.
    # Here, we explicitly set the robot's spawn pose to x=0, y=0, z=0 to center it.
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

    # Launch teleop_twist_keyboard in a new terminal window for keyboard control.
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
