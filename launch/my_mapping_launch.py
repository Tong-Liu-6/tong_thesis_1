import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.3')
    y_pose = LaunchConfiguration('y_pose', default='0.9')

    world = os.path.join(
        get_package_share_directory('my_nav2'),
        'worlds',
        'new_world.model'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    pkg_cartographer = get_package_share_directory('turtlebot3_cartographer')
    cartographer_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_cartographer, 'launch', 'cartographer.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time, 
            'cartographer_config_dir': os.path.join(get_package_share_directory('my_nav2'), 'cartographer_config'),
            'configuration_basename': 'new_turtlebot3_lds_2d.lua'
        }.items()
    )

    bag_play_cmd = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', os.path.join(get_package_share_directory('my_nav2'), 'bags', 'teleop_bag')],
        output='screen',
        shell=True
    )

    save_map_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'nav2_map_server', 'map_saver_cli', 
            '-f', os.path.join(get_package_share_directory('my_nav2'), 'maps', 'new_map')
        ],
        output='screen',
        shell=True
    )

    wait_timer = TimerAction(
            period = 165.0, 
            actions = [save_map_cmd]
        )
    
    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(cartographer_cmd)
    ld.add_action(bag_play_cmd)  
    ld.add_action(wait_timer)  

    return ld
