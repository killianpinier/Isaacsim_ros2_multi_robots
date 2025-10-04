 #!/usr/bin/env python3
"""
Launch RViz visualization for the mycobot robot.

This launch file sets up the complete visualization environment for the mycobot robot,
including robot state publisher, joint state publisher, and RViz2. It handles loading
and processing of URDF/XACRO files and controller configurations.

:author: Addison Sears-Collins
:date: November 15, 2024
"""
import os, yaml
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from panda_utils.urdf_utils import get_robot_description_parameters


def generate_launch_description():
    # Define filenames
    urdf_package = 'panda_description'

    rviz_config_filename = 'panda_description.rviz'
    urdf_parameters_filename = 'urdf_parameters.yaml'

    # Set paths to important files
    pkg_share_description = get_package_share_directory(urdf_package)
    
    default_urdf_filename = 'panda.urdf.xacro'
    default_rviz_config_path = PathJoinSubstitution([pkg_share_description, 'rviz', rviz_config_filename])

    # Launch configuration variables
    jsp_gui                             = LaunchConfiguration('jsp_gui')
    use_jsp                             = LaunchConfiguration('use_jsp')
    use_sim_time                        = LaunchConfiguration('use_sim_time')
    use_rviz                            = LaunchConfiguration('use_rviz')
    rviz_config_file                    = LaunchConfiguration('rviz_config_file')

    namespace                           = LaunchConfiguration('namespace')

    # Declare the launch arguments
    declare_jsp_gui_cmd                 = DeclareLaunchArgument(name='jsp_gui', default_value='true', choices=['true', 'false'], description='Flag to enable joint_state_publisher_gui')
    declare_urdf_model                  = DeclareLaunchArgument(name='urdf_model', default_value=default_urdf_filename)
    declare_use_jsp_cmd                 = DeclareLaunchArgument(name='use_jsp', default_value='false', choices=['true', 'false'], description='Enable the joint state publisher')
    declare_use_sim_time_cmd            = DeclareLaunchArgument(name='use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true')
    declare_use_rviz_cmd                = DeclareLaunchArgument(name='use_rviz', default_value='true', description='Flag to enable RViz')
    declare_rviz_config_file_cmd        = DeclareLaunchArgument(name='rviz_config_file', default_value=default_rviz_config_path, description='Full path to the RVIZ config file to use')
    declare_urdf_yaml_section           = DeclareLaunchArgument(name='urdf_yaml_section', default_value='urdf_mappings')
    declare_namespace                   = DeclareLaunchArgument('namespace', default_value='', description='Namespace')


    def launch_robot_state_publisher(context):

        urdf_package = 'panda_description'

        urdf_parameters_filename = 'urdf_parameters.yaml'

        # Set paths to important files
        pkg_share_description = get_package_share_directory(urdf_package)

        urdf_parameters_file = os.path.join(pkg_share_description, 'config', urdf_parameters_filename)
        urdf_model_path = PathJoinSubstitution([pkg_share_description, 'urdf', LaunchConfiguration("urdf_model").perform(context)])

        xacro_parameters = get_robot_description_parameters(urdf_parameters_file, yaml_section=LaunchConfiguration("urdf_yaml_section").perform(context))
        xacro_command = ["xacro", ' ', urdf_model_path, ' '] + xacro_parameters

        robot_description_content = ParameterValue(Command(xacro_command),  value_type=str)

        # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=LaunchConfiguration('namespace').perform(context),
            name='robot_state_publisher',
            output='both',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_description': robot_description_content
            }]
        )

        return [robot_state_publisher]

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
        declare_urdf_model,
        declare_use_jsp_cmd,
        declare_use_rviz_cmd,
        declare_use_sim_time_cmd,
        declare_urdf_yaml_section,

        declare_namespace,
        joint_state_publisher,
        joint_state_publisher_gui,
        start_rviz_cmd,
        OpaqueFunction(function=launch_robot_state_publisher)
    ])

