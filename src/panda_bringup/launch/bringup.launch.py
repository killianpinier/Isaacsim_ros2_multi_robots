import os
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription

def generate_launch_description():

    package_name_description = "panda_description"
    package_name_bringup = "panda_bringup"
    package_name_moveit_config = "panda_moveit_config"

    pkg_share_description = get_package_share_directory(package_name_description)
    pkg_share_bringup = get_package_share_directory(package_name_bringup)
    pkg_share_moveit_config = get_package_share_directory(package_name_moveit_config)


    declare_use_sim_time    = DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation clock if true")
    declare_prefix          = DeclareLaunchArgument('prefix', default_value='', description='Prefix for robot joints and links')


    use_sim_time            = LaunchConfiguration("use_sim_time")
    prefix                  = LaunchConfiguration("prefix")

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share_moveit_config, 'launch', 'move_group.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'namespace': "panda1",
            'prefix': prefix
        }.items(),
    )


    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share_description, 'launch', 'robot_state_publisher.launch.py')
        ]),
        launch_arguments={
            'jsp_gui': 'false',
            'use_rviz': 'false',
            'use_sim_time': use_sim_time,
            'namespace': "panda1",
            'prefix': prefix
            # 'joint_commands_topic_name': '/panda1/isaac_joint_commands',
            # 'joint_states_topic_name': '/panda1/isaac_joint_states'

        }.items(),
    )

    load_controllers_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share_bringup, 'launch', 'controllers.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'namespace': "panda1"
        }.items(),
    )

    return LaunchDescription([
        declare_use_sim_time,

        move_group,
        robot_state_publisher,
        load_controllers_cmd,
        declare_prefix
    ])