import os
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
from panda_utils.urdf_utils import get_robot_description_parameters

def generate_launch_description():

    package_name_description = "panda_description"
    package_name_bringup = "panda_bringup"
    package_name_moveit_config = "panda_moveit_config"

    pkg_share_description = get_package_share_directory(package_name_description)
    pkg_share_bringup = get_package_share_directory(package_name_bringup)
    pkg_share_moveit_config = get_package_share_directory(package_name_moveit_config)


    declare_panda1_namespace                = DeclareLaunchArgument("panda1_namespace", default_value="panda1")
    declare_panda1_declare_ctrl_file_name   = DeclareLaunchArgument("panda1_ctrl_file_name", default_value="panda1.ros2_controllers.yaml")

    declare_panda2_namespace                = DeclareLaunchArgument("panda2_namespace", default_value="panda2")
    declare_panda2_declare_ctrl_file_name   = DeclareLaunchArgument("panda2_ctrl_file_name", default_value="panda2.ros2_controllers.yaml")

    declare_use_sim_time                    = DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation clock if true")
    declare_prefix                          = DeclareLaunchArgument('prefix', default_value='', description='Prefix for robot joints and links')
    declare_ctrl_file_name                  = DeclareLaunchArgument("controllers_file_name", default_value="ros2_controllers.yaml", description="The filename for the robot's controller")
    declare_eef_type                        = DeclareLaunchArgument('eef_type', default_value='gripper', description='Type of the end-effector')
    declare_use_rviz                        = DeclareLaunchArgument('use_rviz', default_value='false', description='Type of the end-effector')

    use_sim_time                            = LaunchConfiguration("use_sim_time")
    panda1_namespace                        = LaunchConfiguration("panda1_namespace")
    panda2_namespace                        = LaunchConfiguration("panda2_namespace")

    urdf_parameters_filename = 'urdf_parameters.yaml'
    urdf_parameters_file = os.path.join(pkg_share_description, 'config', urdf_parameters_filename)
    urdf_parameters = get_robot_description_parameters(urdf_parameters_file, as_dict=True)

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share_moveit_config, 'launch', 'move_group.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_rviz': LaunchConfiguration('use_rviz')
        }.items(),
    )

    global_robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share_description, 'launch', 'robot_state_publisher.launch.py')
        ]),
        launch_arguments={
            'jsp_gui': 'false',
            'use_rviz': 'false',
            'use_sim_time': use_sim_time,
            'urdf_model': 'dual_panda.urdf.xacro'
        }.items(),
    )

    panda1_robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share_description, 'launch', 'robot_state_publisher.launch.py')
        ]),
        launch_arguments={
            'jsp_gui': 'false',
            'use_rviz': 'false',
            'use_sim_time': use_sim_time,
            'namespace': panda1_namespace,
            'urdf_yaml_section': 'panda1_single_urdf_mappings',
            'urdf_model': 'panda.urdf.xacro'
        }.items(),
    )

    panda2_robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share_description, 'launch', 'robot_state_publisher.launch.py')
        ]),
        launch_arguments={
            'jsp_gui': 'false',
            'use_rviz': 'false',
            'use_sim_time': use_sim_time,
            'namespace': panda2_namespace,
            'urdf_yaml_section': 'panda2_single_urdf_mappings',
            'urdf_model': 'panda.urdf.xacro'
        }.items(),
    )

    panda1_load_controllers_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share_bringup, 'launch', 'controllers.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'namespace': panda1_namespace,
            "controllers_file_name": LaunchConfiguration("panda1_ctrl_file_name"),
            "use_hand_ctrl": 'true' if urdf_parameters["panda1_eef_type"] == "gripper" else 'false'
        }.items(),
    )

    panda2_load_controllers_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share_bringup, 'launch', 'controllers.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'namespace': panda2_namespace,
            "controllers_file_name":  LaunchConfiguration("panda2_ctrl_file_name"),
            "use_hand_ctrl": 'true' if urdf_parameters["panda2_eef_type"] == "gripper" else 'false'
        }.items(),
    )

    # joint_state_merger = Node(
    #     package='panda_bringup',
    #     executable='joint_state_merger',
    #     name='joint_state_merger',
    #     parameters=[
    #         {'robot1_joint_state_topic': '/panda1/joint_states'},
    #         {'robot2_joint_state_topic': '/panda2/joint_states'},
    #         {'output_joint_state_topic': '/joint_states'},
    #         {'publish_rate': 50.0}
    #     ],
    #     output='screen'
    # )

    return LaunchDescription([
        declare_use_sim_time,
        
        declare_panda1_namespace,
        declare_panda1_declare_ctrl_file_name,

        declare_panda2_namespace,
        declare_panda2_declare_ctrl_file_name,

        declare_prefix,
        declare_ctrl_file_name,
        declare_eef_type,
        declare_use_rviz,

        move_group,

        global_robot_state_publisher,
        panda1_robot_state_publisher,
        panda2_robot_state_publisher,

        panda1_load_controllers_cmd,
        panda2_load_controllers_cmd,

        # joint_state_merger
    ])