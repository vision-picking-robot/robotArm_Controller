from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='dsr01'),
        DeclareLaunchArgument('config_dir'),
        DeclareLaunchArgument('db_path'),
        DeclareLaunchArgument('rfid_uid_topic', default_value='/warehouse/rfid/uid'),
        Node(
            package='warehouse_motion', executable='motion_server', output='screen',
            parameters=[{
                'robot_name': LaunchConfiguration('robot_name'),
                'config_dir': LaunchConfiguration('config_dir'),
            }],
        ),
        Node(
            package='warehouse_motion', executable='rfid_db_node', output='screen',
            parameters=[{'db_path': LaunchConfiguration('db_path'), 'seed_demo': False}],
        ),
        Node(
            package='warehouse_motion', executable='picking_coordinator', output='screen',
            parameters=[{'rfid_uid_topic': LaunchConfiguration('rfid_uid_topic')}],
        ),
    ])
