import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    # Declare the use_gui argument
    use_gui_arg = DeclareLaunchArgument(
        name='use_gui', 
        default_value='true', 
        description='Flag to enable joint_state_publisher_gui'
    )

    # Define the robot_description parameter using xacro
    robot_description = Command(
        [
            'xacro ',
            os.path.join(
                os.getenv('AMENT_PREFIX_PATH', '').split(os.pathsep)[0],
                'share/custom_robot_description/urdf/custom_cobot.urdf.xacro'
            )
        ]
    )

    # Nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d',
            os.path.join(
                os.getenv('AMENT_PREFIX_PATH', '').split(os.pathsep)[0],
                'share/custom_robot_description/config/config.rviz'
            )
        ]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('use_gui'))
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_gui'))
    )

    ld.add_action(use_gui_arg)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(rviz_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(joint_state_publisher_gui_node)

    return ld
