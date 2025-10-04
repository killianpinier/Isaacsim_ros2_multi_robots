import os
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration, TextSubstitution, PathJoinSubstitution
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from panda_utils.urdf_utils import get_robot_description_parameters
from panda_utils.srdf_utils import get_robot_description_semantic_parameters

def generate_launch_description():

    robot_description_xacro_file_path = os.path.join(
        get_package_share_directory("panda_description"),
        "urdf",
        "dual_panda.urdf.xacro"
    )

    declare_namespace                   = DeclareLaunchArgument("namespace", default_value="", description="The namespace of the robot")
    declare_ros2_control_hardware_type  = DeclareLaunchArgument("ros2_control_hardware_type", default_value="isaac", description="ROS2 control hardware interface type to use for the launch file -- possible values: [mock_components, isaac]")
    declare_use_sim_time                = DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation clock if true")
    declare_use_rviz                    = DeclareLaunchArgument('use_rviz', default_value='false')


    pkg_share_description = get_package_share_directory("panda_description")
    pkg_share_moveit_config = get_package_share_directory("panda_moveit_config")

    config_path = os.path.join(pkg_share_moveit_config, "config", "panda1_gripper_panda2_peeler")

    urdf_parameters_filename = 'urdf_parameters.yaml'
    urdf_parameters_file = os.path.join(pkg_share_description, 'config', urdf_parameters_filename)

    srdf_parameters_filename = 'srdf_parameters.yaml'
    srdf_parameters_file = os.path.join(config_path, srdf_parameters_filename)

    joint_limits_file_path          = os.path.join(config_path, "joint_limits.yaml")
    kinematics_file_path            = os.path.join(config_path, "kinematics.yaml")
    moveit_ctrl_file_path           = os.path.join(config_path, "gripper_moveit_controllers.yaml")
    srdf_xacro_file_path                  = os.path.join(config_path, "panda.srdf.xacro")
    pilz_cartesian_limits_file_path = os.path.join(config_path, "pilz_cartesian_limits.yaml")

    xacro_urdf_parameters = get_robot_description_parameters(urdf_parameters_file, as_dict=True)
    xacro_srdf_parameters = get_robot_description_semantic_parameters(srdf_parameters_file)


    moveit_config = (
        MoveItConfigsBuilder("panda")
        .robot_description(
            file_path=robot_description_xacro_file_path,
            mappings=xacro_urdf_parameters
        )
        .robot_description_semantic(
            file_path=srdf_xacro_file_path,
            mappings=xacro_srdf_parameters
        )
        .trajectory_execution(file_path=moveit_ctrl_file_path)
        .joint_limits(file_path=joint_limits_file_path)
        .robot_description_kinematics(file_path=kinematics_file_path)
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"], default_planning_pipeline="ompl")
        .pilz_cartesian_limits(file_path=pilz_cartesian_limits_file_path)
        .to_moveit_configs()
    )

    # MoveIt capabilities
    move_group_capabilities = {"capabilities": "move_group/ExecuteTaskSolutionCapability"}

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        # namespace=namespace,
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            move_group_capabilities
        ],
        # arguments=["--ros-args", "--log-level", "info"],
    )

    rviz_config_file = PathJoinSubstitution([
        get_package_share_directory("panda_moveit_config"),
        "rviz",
        "panda_moveit_config.rviz"
    ])


    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        condition=IfCondition(LaunchConfiguration("use_rviz"))
    )

    return LaunchDescription([
        declare_ros2_control_hardware_type,
        declare_use_sim_time,
        declare_namespace,
        declare_use_rviz,

        move_group_node, 
        rviz_node,
    ])