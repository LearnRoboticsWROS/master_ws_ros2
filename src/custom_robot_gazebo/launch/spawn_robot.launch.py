from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Percorsi ai file necessari
    robot_description_file = os.path.join(
        get_package_share_directory('custom_robot_description'), 'urdf', 'custom_cobot.urdf.xacro'
    )
    joint_controllers_file = os.path.join(
        get_package_share_directory('custom_robot_gazebo'), 'config', 'joint_controllers.yaml'
    )
    gazebo_launch_file = os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
    )

    # Parametro robot_description
    robot_description = Command(['xacro ', robot_description_file])

    # Dichiarazioni degli argomenti
    x_arg = DeclareLaunchArgument('x', default_value='0', description='X position of the robot')
    y_arg = DeclareLaunchArgument('y', default_value='0', description='Y position of the robot')
    z_arg = DeclareLaunchArgument('z', default_value='0', description='Z position of the robot')

    # Include Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={
            'use_sim_time': 'true',
            'debug': 'false',
            'gui': 'true',
            'paused': 'true',
        }.items()
    )

    # Nodo per spawnare il robot
    spawn_the_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'cobot',
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z')
        ],
        output='screen'
    )


    controller_manager_params = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace='/cobot',
        parameters=[joint_controllers_file],
        output='both',
        remappings=[
            ("/robot_description", "/cobot/robot_description"),
        ],
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='both',
        remappings=[('/joint_states', '/cobot/joint_states')]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/cobot/controller_manager"],
    )



    # Nodo per avviare i controller
    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace='/cobot',
        arguments=["arm_position_controller", "--controller-manager", "/cobot/controller_manager"],
    )

    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen'
    )



    # RQT GUI
    rqt_reconfigure = Node(
        package='rqt_gui',
        executable='rqt_gui',
        output='screen'
    )

    # Launch Description
    ld = LaunchDescription()
    ld.add_action(x_arg)
    ld.add_action(y_arg)
    ld.add_action(z_arg)
    ld.add_action(gazebo)
    ld.add_action(spawn_the_robot)
    ld.add_action(controller_manager_params)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(controller_spawner)
    ld.add_action(joint_state_publisher)
    ld.add_action(robot_state_publisher)
    ld.add_action(rqt_reconfigure)

    return ld
