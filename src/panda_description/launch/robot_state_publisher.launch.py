 #!/usr/bin/env python3
"""
Launch RViz visualization for the mycobot robot.

This launch file sets up the complete visualization environment for the mycobot robot,
including robot state publisher, joint state publisher, and RViz2. It handles loading
and processing of URDF/XACRO files and controller configurations.

:author: Addison Sears-Collins
:date: November 15, 2024
"""
import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


# Define the arguments for the XACRO file
# ARGUMENTS = [
#     DeclareLaunchArgument('robot_name', default_value='mycobot_280', description='Name of the robot'),
#     DeclareLaunchArgument('namespace', default_value='', description='Namespace'),
#     DeclareLaunchArgument('prefix', default_value='', description='Prefix for robot joints and links'),
#     DeclareLaunchArgument('add_world', default_value='true', choices=['true', 'false'], description='Whether to add world link'),
#     DeclareLaunchArgument('eef_type', default_value='adaptive_gripper', description='Type of the end-effector'),
#     DeclareLaunchArgument('use_eef', default_value='true', choices=['true', 'false'], description='Whether to attach an end-effector'),
# ]


def generate_launch_description():
    # Define filenames
    urdf_package = 'panda_description'
    urdf_filename = 'panda.urdf.xacro'

    rviz_config_filename = 'panda_description.rviz'

    # Set paths to important files
    pkg_share_description = FindPackageShare(urdf_package)
    default_urdf_model_path = PathJoinSubstitution([pkg_share_description, 'urdf', urdf_filename])
    default_rviz_config_path = PathJoinSubstitution([pkg_share_description, 'rviz', rviz_config_filename])

    # Launch configuration variables
    jsp_gui                             = LaunchConfiguration('jsp_gui')
    urdf_model                          = LaunchConfiguration('urdf_model')
    use_jsp                             = LaunchConfiguration('use_jsp')
    use_sim_time                        = LaunchConfiguration('use_sim_time')
    use_rviz                            = LaunchConfiguration('use_rviz')
    rviz_config_file                    = LaunchConfiguration('rviz_config_file')

    # robot_name                          = LaunchConfiguration('robot_name')
    # prefix                              = LaunchConfiguration('prefix')
    namespace                           = LaunchConfiguration('namespace')
    x                                   = LaunchConfiguration('x')
    y                                   = LaunchConfiguration('y')
    z                                   = LaunchConfiguration('z')
    yaw                                 = LaunchConfiguration('yaw')
    pitch                               = LaunchConfiguration('pitch')
    roll                                = LaunchConfiguration('roll')


    # Declare the launch arguments
    declare_jsp_gui_cmd                 = DeclareLaunchArgument(name='jsp_gui', default_value='true', choices=['true', 'false'], description='Flag to enable joint_state_publisher_gui')
    declare_urdf_model_path_cmd         = DeclareLaunchArgument(name='urdf_model', default_value=default_urdf_model_path,description='Absolute path to robot urdf file')
    declare_use_jsp_cmd                 = DeclareLaunchArgument(name='use_jsp', default_value='false', choices=['true', 'false'], description='Enable the joint state publisher')
    declare_use_sim_time_cmd            = DeclareLaunchArgument(name='use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true')
    declare_use_rviz_cmd                = DeclareLaunchArgument(name='use_rviz', default_value='true', description='Flag to enable RViz')
    declare_rviz_config_file_cmd        = DeclareLaunchArgument(name='rviz_config_file', default_value=default_rviz_config_path, description='Full path to the RVIZ config file to use')

    declare_namespace                   = DeclareLaunchArgument('namespace', default_value='', description='Namespace')
    declare_joint_commands_topic_name   = DeclareLaunchArgument('joint_commands_topic_name', default_value='isaac_joint_commands', description='Joint_commands_topic_name')
    declare_joint_states_topic_name     = DeclareLaunchArgument('joint_states_topic_name', default_value='isaac_joint_states', description='Joint_states_topic_name')
    declare_x                           = DeclareLaunchArgument('x', default_value='0.0')
    declare_y                           = DeclareLaunchArgument('x', default_value='0.0')
    declare_z                           = DeclareLaunchArgument('x', default_value='0.0')
    declare_yaw                         = DeclareLaunchArgument('x', default_value='0.0')
    declare_pitch                       = DeclareLaunchArgument('x', default_value='0.0')
    declare_roll                        = DeclareLaunchArgument('x', default_value='0.0')


    declare_ros2_control_hardware_type = DeclareLaunchArgument(
        "ros2_control_hardware_type",
        default_value="isaac",
        description="ROS2 control hardware interface type to use for the launch file -- possible values: [mock_components, isaac]",
    )


    robot_description_content = ParameterValue(Command([
        'xacro', ' ', urdf_model, ' ',
        'ros2_control_hardware_type:=', LaunchConfiguration("ros2_control_hardware_type"), ' ',
        'joint_commands_topic_name:=', LaunchConfiguration("joint_commands_topic_name"), ' ',
        'joint_states_topic_name:=', LaunchConfiguration("joint_states_topic_name"), ' ',
        # 'prefix:=', LaunchConfiguration('add_world'), ' ',
        # 'add_world:=',          LaunchConfiguration('add_world'), ' ',
        # 'base_link:=',          LaunchConfiguration('base_link'), ' ',
        # 'base_type:=',          LaunchConfiguration('base_type'), ' ',
        # 'flange_link:=',        LaunchConfiguration('flange_link'), ' ',
        # 'eef_type:=',           LaunchConfiguration('eef_type'), ' ',
        # 'use_camera:=',         LaunchConfiguration('use_camera'), ' ',
        # 'use_gazebo:=',         LaunchConfiguration('use_gazebo'), ' ',
        # 'use_eef:=',            LaunchConfiguration('use_eef'), ' ',
        # 'namespace:=',          LaunchConfiguration('namespace'), ' ',
    ]), value_type=str)

    # robot_description = {"robot_description": robot_description_content}
    # robot_controllers = PathJoinSubstitution(
    #     [
    #         FindPackageShare('mycobot_moveit_config'),
    #         'config',
    #         robot_name,
    #         'ros2_controllers.yaml',
    #     ]
    # )

    # Static TF
    world2robot_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher_world_to_robot",
        output="log",
        arguments=[
            "--x", x,
            "--y", y,
            "--z", z,
            "--yaw", yaw,
            "--pitch", pitch,
            "--roll", roll,
            "--frame-id", "World",
            "--child-frame-id", "panda_link0"
        ],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        name='robot_state_publisher',
        output='both',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_content
        }]
    )

    # Publish the joint state values for the non-fixed joints in the URDF file.
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        namespace=namespace,
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_jsp))

    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        namespace=namespace,
        name='joint_state_publisher_gui',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(jsp_gui))

    # Launch RViz
    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        namespace=namespace,
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Create the launch description and populate
    return LaunchDescription([
        declare_jsp_gui_cmd,
        declare_rviz_config_file_cmd,
        declare_urdf_model_path_cmd,
        declare_use_jsp_cmd,
        declare_use_rviz_cmd,
        declare_use_sim_time_cmd,
        declare_ros2_control_hardware_type,
        declare_namespace,
        declare_joint_states_topic_name,
        declare_joint_commands_topic_name,

        declare_x,
        declare_y,
        declare_z,
        declare_yaw,
        declare_roll,
        declare_pitch,

        world2robot_tf_node,
        robot_state_publisher,
        joint_state_publisher,
        joint_state_publisher_gui,
        start_rviz_cmd,
    ])

