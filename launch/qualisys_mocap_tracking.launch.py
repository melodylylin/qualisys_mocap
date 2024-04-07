from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    mocap_server_ip = LaunchConfiguration('mocap_server_ip')
    mocap_server_ip_arg = DeclareLaunchArgument(
        'mocap_server_ip',
        default_value='192.168.123.2'
    )
    mocap_node = Node(
        package='qualisys_mocap',
        executable='qualisys_node',
        parameters=[
                {'mocap_server_ip': mocap_server_ip},
            ] 		
    )

    return LaunchDescription([
        mocap_server_ip_arg,
        mocap_node,
    ])
