from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Paths to the external launch files
    rplidar_launch_path = os.path.join(
        get_package_share_directory('rplidar_ros'), 'launch', 'rplidar_a1_launch.py'
    )
    bno055_launch_path = os.path.join(
        get_package_share_directory('bno055'), 'launch', 'bno055_launch.py'  #
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_lidar', default_value='true', description='Launch LiDAR'),
        DeclareLaunchArgument('use_imu', default_value='true', description='Launch IMU'),
        DeclareLaunchArgument('use_serial', default_value='true', description='Launch serial com'),

        Node(
            package='robin_SAS',
            executable='teleop_to_serial',
            name='teleop_to_serial',
            condition=IfCondition(LaunchConfiguration('use_serial'))
        ),

        # LiDAR (includes the rplidar launch file)
      #  Node(
     #       package='tf2_ros',
    #        executable='static_transform_publisher',
   #         name='world_to_base_link_tf',
  #          arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link'],
 #           condition=IfCondition(LaunchConfiguration('use_lidar'))
#        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rplidar_launch_path),
            condition=IfCondition(LaunchConfiguration('use_lidar'))
        ),

        # IMU (includes the bno055 node with parameters)
      #  Node(
     #       package='tf2_ros',
    #        executable='static_transform_publisher',
   #         name='base_link_to_bno055_tf',
  #          arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'bno055'],
 #           condition=IfCondition(LaunchConfiguration('use_imu'))
#        ),
        Node(
            package='bno055',
            executable='bno055',
            name='bno055',
            parameters=[{
                'connection_type': 'i2c',
                'i2c_bus': 1,
                'i2c_address': '0x28'
            }],
            condition=IfCondition(LaunchConfiguration('use_imu'))
        )
    ])
