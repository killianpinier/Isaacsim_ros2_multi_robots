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
    declare_namespace       = DeclareLaunchArgument('namespace', default_value='', description='Namespace to use for the robot')
    declare_ctrl_file_name  = DeclareLaunchArgument("controllers_file_name", default_value="ros2_controllers.yaml", description="The filename for the robot's controller")
    declare_eef_type        = DeclareLaunchArgument('eef_type', default_value='gripper', description='Type of the end-effector')
    declare_use_rviz        = DeclareLaunchArgument('use_rviz', default_value='false', description='Type of the end-effector')

    use_sim_time            = LaunchConfiguration("use_sim_time")
    prefix                  = LaunchConfiguration("prefix")
    namespace               = LaunchConfiguration("namespace")
    ctrl_file_name          = LaunchConfiguration("ctrl_file_name")
    eef_type                = LaunchConfiguration('eef_type')

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share_moveit_config, 'launch', 'move_group.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            # 'namespace': namespace,
            'prefix': prefix,
            'eef_type': eef_type,
            'use_rviz': LaunchConfiguration('use_rviz')
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
            # 'namespace': namespace,
            'prefix': prefix,
            'eef_type': eef_type
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
            'namespace': namespace,
            "controllers_file_name": ctrl_file_name
        }.items(),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_namespace,
        declare_prefix,
        declare_ctrl_file_name,
        declare_eef_type,
        declare_use_rviz,

        move_group,
        robot_state_publisher,
        load_controllers_cmd,
    ])