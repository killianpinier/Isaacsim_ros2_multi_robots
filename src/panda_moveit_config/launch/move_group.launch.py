import os
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, TextSubstitution, PathJoinSubstitution
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

def generate_launch_description():

    robot_description_xacro_file_path = os.path.join(
        get_package_share_directory("panda_description"),
        "urdf",
        "panda.urdf.xacro"
    )

    declare_namespace                   = DeclareLaunchArgument("namespace", default_value="", description="The namespace of the robot")
    declare_ros2_control_hardware_type  = DeclareLaunchArgument("ros2_control_hardware_type", default_value="isaac", description="ROS2 control hardware interface type to use for the launch file -- possible values: [mock_components, isaac]")
    declare_prefix                      = DeclareLaunchArgument('prefix', default_value='', description='Prefix for robot joints and links')
    declare_use_sim_time                = DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation clock if true")

    namespace = LaunchConfiguration("namespace")

    moveit_config = (
        MoveItConfigsBuilder("panda")
        .robot_description(
            file_path=robot_description_xacro_file_path,
            mappings={
                "prefix": LaunchConfiguration("prefix"),
                "ros2_control_hardware_type": LaunchConfiguration("ros2_control_hardware_type"),
            },
        )
        .robot_description_semantic(file_path="config/panda.srdf")
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )

    # MoveIt capabilities
    move_group_capabilities = {"capabilities": "move_group/ExecuteTaskSolutionCapability"}

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace=namespace,
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            move_group_capabilities
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # RViz
    rviz_config_file = PathJoinSubstitution([
        get_package_share_directory("panda_moveit_config"),
        "rviz",
        [TextSubstitution(text="panda_moveit_config_ns_"), LaunchConfiguration("namespace"), TextSubstitution(text=".rviz")]
    ])


    get_package_share_directory

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
    )

    return LaunchDescription([
        declare_ros2_control_hardware_type,
        declare_use_sim_time,
        declare_namespace,
        declare_prefix,

        move_group_node,
        rviz_node
    ])