from ament_index_python.packages import get_package_share_directory
import os
import launch
import launch_ros.actions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    cerveza_localization_pkg_path = get_package_share_directory('cerveza_tb_localization')
    cerveza_planning_pkg_path = get_package_share_directory('cerveza_tb_planning')

    #Joy teleop launch file path
    joy_teleop_launch_path = os.path.join(cerveza_localization_pkg_path, 'launch', 'turtlebot_joy_teleop.launch.py')

    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="cerveza_tb_planning",
                executable="control_tb",
                name="control_tb",
                namespace="turtlebot",
            ),
            launch_ros.actions.Node(
                package="cerveza_tb_planning",
                executable="grid_mapping",
                name="grid_mapping",
                namespace="turtlebot",
                parameters=[
                    {"is_sim": False}
                ]
            ),
            launch_ros.actions.Node(
                package="cerveza_tb_planning",
                executable="rrt_motion_planning",
                name="rrt_motion_planning",
                namespace="turtlebot",
            ),
            launch_ros.actions.Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='world_to_odom_publisher',
                arguments=[
                    '--x', '0', '--y', '0', '--z', '0',
                    '--yaw', '0', '--pitch', '0', '--roll', '0',
                    '--frame-id', 'world_enu',
                    '--child-frame-id', 'odom'
                ]
            ),
            launch_ros.actions.Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=['-d', PathJoinSubstitution([cerveza_planning_pkg_path, 'config', 'tb_map_rviz_conf.rviz'])]
            )
        ]
    )
