import os
import launch
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
import launch_ros



def generate_launch_description():
    pkgPath = launch_ros.substitutions.FindPackageShare(package='mass515').find('robot_description')
    urdfModelPath = os.path.join(pkgPath, 'urdf/model.urdf')
    
    # finding the path for rviz configuration
    rviz_config_path  = os.path.join(get_package_share_path('robot_description'), 'rviz', 'rviz_config.rviz')

    with open(urdfModelPath, 'r') as infp:
        robot_desc = infp.read()
    
    params = {'robot_description': robot_desc}


    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    
    # static_transform_publisher = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='world_to_map_broadcaster',
    #     output='screen',
    #     arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom']
    #)
    
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[params],
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )
    

    # joint_state_publisher_gui_node = launch_ros.actions.Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     name='joint_state_publisher_gui',
    #     condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    # )


    rviz2_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )
 
        # Add odometry node
    odometry_node = launch_ros.actions.Node(
        package='odometry',
        executable='odometry_publisher',  # Make sure this matches your setup.py entry point
        name='odometry_node',
        output='screen'
    )
    

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                              description='This is a flag for joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=urdfModelPath,
                                              description='Path to the urdf model file'),
        robot_state_publisher_node,
        joint_state_publisher_node,
        # joint_state_publisher_gui_node,
        rviz2_node,
        odometry_node,  # Include the odometry node in the launch description
        #static_transform_publisher
    ])