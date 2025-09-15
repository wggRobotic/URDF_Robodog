from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    # Default Pfade
    urdf_pkg = 'urdf_robodog'
    urdf_file = 'urdf/Robodog.urdf'
    rviz_file = 'rviz/urdf.rviz'

    urdf_path = os.path.join(
        os.path.dirname(__file__),
        '..',
        urdf_file
    )
    rviz_path = os.path.join(
        os.path.dirname(__file__),
        '..',
        rviz_file
    )

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # Launch arguments (optional)
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='false',
        choices=['true', 'false'],
        description='Use joint_state_publisher_gui'
    )

    # Nodes
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
        output='screen'
    )

    # Dein eigener Joint-State-Publisher (falls in deinem Package `robodog_control`)
    joint_pub_node = Node(
        package='urdf_robodog',
        executable='joint_state_node',
        name='joint_publisher',
        output='screen'
    )

    # Optional: direkt RViz starten
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_path],
        output='screen'
    )

    return LaunchDescription([
        gui_arg,
        rsp_node,
        joint_pub_node,
        rviz_node
    ])
