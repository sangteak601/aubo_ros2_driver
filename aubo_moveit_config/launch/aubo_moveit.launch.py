from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
import yaml
from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def launch_setup(context, *args, **kwargs):
    # Command-line arguments
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    ip = LaunchConfiguration("ip")
    use_real_hardware = LaunchConfiguration("use_real_hardware")

    moveit_config = (
        MoveItConfigsBuilder(robot_name="aubo_iS10", package_name=moveit_config_package)
        .robot_description(
            file_path="config/aubo_iS10.urdf.xacro",
            mappings={"use_real_hardware": use_real_hardware, "ip": ip},
        )
        .robot_description_semantic(file_path="config/aubo_iS10.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines("pilz_industrial_motion_planner", pipelines=["pilz_industrial_motion_planner"])
        .pilz_cartesian_limits(file_path="config/pilz_cartesian_limits.yaml")
        .to_moveit_configs()
    )
    moveit_config.move_group_capabilities = {
        "capabilities": "pilz_industrial_motion_planner/MoveGroupSequenceAction pilz_industrial_motion_planner/MoveGroupSequenceService"
    }

    move_group_node = launch_ros.actions.Node(
        package="moveit_ros_move_group", executable="move_group", output="screen", parameters=[moveit_config.to_dict()]
    )

    # RViz
    rviz_base = os.path.join(
        get_package_share_directory("aubo_moveit_config"), "config"
    )
    rviz_config = os.path.join(rviz_base, "view_robot.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    # # Static TF
    # static_tf_node = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="static_transform_publisher",
    #     output="log",
    #     arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    # )

    # Publish TF
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    robot_controllers = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "config", "ros2_controllers.yaml"]
    )

    ros2_control_node = Node(
        package="aubo_ros2_driver",
        executable="aubo_ros2_control_node",
        parameters=[robot_controllers],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--param-file", robot_controllers],
    )

    nodes_to_start = [move_group_node, rviz_node, robot_state_pub_node, ros2_control_node, joint_state_broadcaster_spawner, arm_controller_spawner]
    return nodes_to_start


def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="aubo_moveit_config",
            description="Name of the support package",
            choices=["aubo_moveit_config"],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_real_hardware",
            default_value="false",
            description="Use real hardware",
            choices=["true", "false"],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ip",
            default_value="192.168.10.2",
        )
    )
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
