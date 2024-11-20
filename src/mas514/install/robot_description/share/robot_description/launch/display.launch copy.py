import os
import launch
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch_ros.actions import Node, LifecycleNode
from launch.substitutions import Command, LaunchConfiguration
import launch_ros


from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    RegisterEventHandler,
    IncludeLaunchDescription,
)
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from launch.substitutions import (
    AndSubstitution,
    LaunchConfiguration,
    NotSubstitution,
)
from launch.conditions import IfCondition
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource

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
    autostart = LaunchConfiguration("autostart")
    use_lifecycle_manager = LaunchConfiguration("use_lifecycle_manager")
    use_sim_time = LaunchConfiguration("use_sim_time")
    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description=(
            "Automatically startup the SLAM Toolbox. "
            "Ignored when use_lifecycle_manager is true."
        ),
    )
    declare_use_lifecycle_manager = DeclareLaunchArgument(
        "use_lifecycle_manager",
        default_value="false",
        description="Enable bond connection during node activation",
    )
    declare_use_sim_time_argument = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation/Gazebo clock",
    )
    slam_toolbox_node = LifecycleNode(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        namespace="",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "odom_frame": "odom",
                "map_frame": "map",
                "base_frame": "base_link",
                "scan_topic": "/scan",
                "resolution": 0.05,
                "mode": "localization",
                "map_file_name": "/home/asle/mas515v2/of3",
                "map_start_at_dock": "true"
            }
        ],
    )
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=lambda n: n == slam_toolbox_node,
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(
            AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager))
        ),
    )
    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="[LifecycleLaunch] Activating SLAM Toolbox node..."),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=lambda n: n == slam_toolbox_node,
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        ),
        condition=IfCondition(
            AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager))
        ),
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
        slam_toolbox_node,
        declare_autostart_cmd,
        declare_use_lifecycle_manager,
        declare_use_sim_time_argument,
        configure_event,
        activate_event
    ])