#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Constants for paths to different files and folders
    package_name_description = 'mycobot_description'
    package_name_moveit_config = 'mycobot_moveit_config'

    # Get the package share directory
    pkg_share_moveit_config_temp = FindPackageShare(package=package_name_moveit_config)

    # Launch configuration variables
    use_sim_time                = LaunchConfiguration('use_sim_time')
    exe                         = LaunchConfiguration('exe')

    prefix                      = LaunchConfiguration('prefix')
    namespace                   = LaunchConfiguration('namespace')
    use_eef                     = LaunchConfiguration('use_eef')
    eef_type                    = LaunchConfiguration('eef_type')
    # srdf_config_file            = LaunchConfiguration('srdf_config_file')

    # Declare the launch arguments
    # declare_robot_name_cmd      = DeclareLaunchArgument(name='robot_name', default_value='mycobot_280', description='Name of the robot to use')
    # declare_prefix_cmd          = DeclareLaunchArgument(name='prefix', default_value='', description='Prefix of the robot to use')
    declare_namespace_cmd       = DeclareLaunchArgument(name='namespace', default_value='', description='Namespace of the robot to use')
    declare_use_sim_time_cmd    = DeclareLaunchArgument(name='use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true')
    declare_exe_cmd             = DeclareLaunchArgument(name='exe', default_value='', description='Name of the executable to run')
    declare_use_eef_cmd         = DeclareLaunchArgument(name='use_eef', default_value='true', description='Name of the robot to use')
    declare_eef_type_cmd        = DeclareLaunchArgument(name='eef_type', default_value='adaptive_gripper', description='Use simulation (Gazebo) clock if true')
    declare_srdf_cfg_file_cmd   = DeclareLaunchArgument(name='srdf_config_file', default_value='mycobot_280_gripper.srdf', description='SRDF config file name')

    declare_ros2_control_hardware_type = DeclareLaunchArgument(
        "ros2_control_hardware_type",
        default_value="isaac",
        description="ROS2 control hardware interface type to use for the launch file -- possible values: [mock_components, isaac]",
    )

    # declare_exe_cmd = DeclareLaunchArgument(
    #     name="exe",
    #     default_value="alternative_path_costs",
    #     description="Which demo to run",
    #     choices=["alternative_path_costs", "cartesian", "fallbacks_move_to",
    #              "ik_clearance_cost", "modular", "manual_pick_place"])

    def configure_setup(context):
        """Configure MoveIt and create nodes with proper string conversions."""
        # # Get the robot name as a string for use in MoveItConfigsBuilder
        # robot_name_str                  = LaunchConfiguration('robot_name').perform(context)
        # srdf_config_file_str            = LaunchConfiguration('srdf_config_file').perform(context)

        # # Get package path
        # pkg_share_moveit_config         = pkg_share_moveit_config_temp.find(package_name_moveit_config)
        # pkg_share_description           = pkg_share_moveit_config_temp.find(package_name_description)

        # # Construct file paths using robot name string
        # config_path                     = os.path.join(pkg_share_moveit_config, 'config', robot_name_str)

        # # Define all config file paths
        # initial_positions_file_path     = os.path.join(config_path, 'initial_positions.yaml')
        # joint_limits_file_path          = os.path.join(config_path, 'joint_limits.yaml')
        # kinematics_file_path            = os.path.join(config_path, 'kinematics.yaml')
        # moveit_controllers_file_path    = os.path.join(config_path, 'moveit_controllers.yaml')
        # srdf_model_path                 = os.path.join(config_path, srdf_config_file_str)
        # pilz_cartesian_limits_file_path = os.path.join(config_path, 'pilz_cartesian_limits.yaml')

        # robot_description_xacro_file = os.path.join(
        #     pkg_share_description, 'urdf', 'robots', f'mycobot_280.urdf.xacro'
        # )

        # Create MoveIt configuration
        # moveit_config = (
        #     MoveItConfigsBuilder(robot_name_str, package_name=package_name_moveit_config)
        #     .robot_description(
        #         file_path=robot_description_xacro_file,
        #         mappings={
        #             'use_eef': use_eef,
        #             'eef_type': eef_type,
        #             'robot_name': robot_name_str,
        #             'namespace': namespace,
        #             'prefix': prefix
        #         }
        #     )
        #     .trajectory_execution(file_path=moveit_controllers_file_path)
        #     .robot_description_semantic(file_path=srdf_model_path)
        #     .joint_limits(file_path=joint_limits_file_path)
        #     .robot_description_kinematics(file_path=kinematics_file_path)
        #     .planning_pipelines(
        #         pipelines=["ompl", "pilz_industrial_motion_planner", "stomp"],
        #         default_planning_pipeline="ompl"
        #     )
        #     .planning_scene_monitor(
        #         publish_robot_description=False,
        #         publish_robot_description_semantic=False,
        #         publish_planning_scene=False,
        #     )
        #     .pilz_cartesian_limits(file_path=pilz_cartesian_limits_file_path)
        #     .to_moveit_configs()
        # )

        robot_description_xacro_file_path = os.path.join(
            get_package_share_directory("panda_description"),
            "urdf",
            "panda.urdf.xacro"
        )

        moveit_config = (
            MoveItConfigsBuilder("panda")
            .robot_description(
                file_path=robot_description_xacro_file_path,
                mappings={
                    "ros2_control_hardware_type": LaunchConfiguration("ros2_control_hardware_type"),
                },
            )
            .robot_description_semantic(file_path="config/panda.srdf")
            .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
            .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
            .to_moveit_configs()
        )

        # Create MTC node
        mtc_node = Node(
            package="mtc_tests",
            executable=exe,
            namespace=namespace,
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                {'use_sim_time': use_sim_time},
                # {'start_state': {'content': initial_positions_file_path}}
            ],
        )

        return [mtc_node]
    
    robot_description_xacro_file_path = os.path.join(
            get_package_share_directory("panda_description"),
            "urdf",
            "panda.urdf.xacro"
        )
    
    moveit_config = (
        MoveItConfigsBuilder("panda")
        .robot_description(
            file_path=robot_description_xacro_file_path,
            mappings={
                "ros2_control_hardware_type": LaunchConfiguration("ros2_control_hardware_type"),
            },
        )
        .robot_description_semantic(file_path="config/panda.srdf")
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    # Create MTC node
    mtc_node = Node(
        package="mtc_tests",
        executable=exe,
        namespace=namespace,
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': use_sim_time},
            # {'start_state': {'content': initial_positions_file_path}}
        ],
    )

    # Create the launch description
    ld = LaunchDescription()

    # Add the launch arguments
    ld.add_action(declare_ros2_control_hardware_type)
    # ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_eef_cmd)
    ld.add_action(declare_eef_type_cmd)
    # ld.add_action(declare_prefix_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_exe_cmd)
    ld.add_action(declare_srdf_cfg_file_cmd)

    ld.add_action(mtc_node)

    # Add the setup and node creation
    # ld.add_action(OpaqueFunction(function=configure_setup))

    return ld
