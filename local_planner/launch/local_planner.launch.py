import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node  # Import the Node action

def generate_launch_description():
    # launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    # pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # x_pose = LaunchConfiguration('x_pose', default='0.0')
    # y_pose = LaunchConfiguration('y_pose', default='0.0')

    # # World file
    # world = os.path.join(
    #     get_package_share_directory('turtlebot3_gazebo'),
    #     'worlds',
    #     'empty_world.world'
    # )

    # # Launch commands
    # gzserver_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
    #     ),
    #     launch_arguments={'world': world}.items()
    # )

    # gzclient_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
    #     )
    # )

    # robot_state_publisher_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
    #     ),
    #     launch_arguments={'use_sim_time': use_sim_time}.items()
    # )

    # spawn_turtlebot_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
    #     ),
    #     launch_arguments={
    #         'x_pose': x_pose,
    #         'y_pose': y_pose
    #     }.items()
    # )

    # Additional nodes from 'my_package'
    node1 = Node(
        package='local_planner',
        executable='ipc_controller.py',
        name='ipc_controller',
        output='screen',
    )

    node2 = Node(
        package='local_planner',
        executable='obstacles.py',
        name='obstacles',
        output='screen',
    )

    node3 = Node(
        package='local_planner',
        executable='waypoints.py',
        name='waypoints',
        output='screen',
    )

    # RViz node with config
    rviz_config_file = os.path.join(
        get_package_share_directory('local_planner'),
        'config',
        'localplnner.rviz'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    # Launch description
    ld = LaunchDescription()

    # Add the commands to the launch description
    # ld.add_action(gzserver_cmd)
    # ld.add_action(gzclient_cmd)
    # ld.add_action(robot_state_publisher_cmd)
    # ld.add_action(spawn_turtlebot_cmd)

    # Add the additional nodes
    ld.add_action(node1)
    ld.add_action(node2)
    ld.add_action(node3)

    # Add RViz
    ld.add_action(rviz_node)

    return ld