from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)


def launch_setup(context, *args, **kwargs):
    # Initialize Arguments
    node_namespace = LaunchConfiguration("node_namespace")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("netft_utils"), "urdf", "example_description.urdf.xacro"]
            ),
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    control_configs = PathJoinSubstitution(
        [FindPackageShare("netft_utils"), "config", "control_params.yaml"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=node_namespace,
        parameters=[
            robot_description,
            ParameterFile(control_configs, allow_substs=True),
        ],
        output="screen",
    )

    ft_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        namespace=node_namespace,
        arguments=[
            "force_torque_sensor_broadcaster",
            "--controller-manager",
            "controller_manager",
            "-n",
            node_namespace,
            "--controller-manager-timeout",
            "10",
        ],
        output="screen",
    )

    nodes_to_start = [
        control_node,
        ft_broadcaster]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "node_namespace",
            default_value="/",
            description="The namespace to put all the nodes into",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
