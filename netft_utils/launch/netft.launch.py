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
    address = LaunchConfiguration("address")
    # tf_prefix = LaunchConfiguration("tf_prefix")
    node_namespace = LaunchConfiguration("node_namespace")
    # use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    # controller_spawner_timeout = LaunchConfiguration("controller_spawner_timeout")
    # activate_joint_controller = LaunchConfiguration("activate_joint_controller")
    # launch_dashboard_client = LaunchConfiguration("launch_dashboard_client")
    # use_tool_communication = LaunchConfiguration("use_tool_communication")
    # tool_device_name = LaunchConfiguration("tool_device_name")
    # tool_tcp_port = LaunchConfiguration("tool_tcp_port")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("table_ur20_description"), "urdf", "han.urdf.xacro"]
            ),
            " ",
            "use_fake_hardware:=",
            "true",
            " ",
            "fake_sensor_commands:=",
            "true",
            " ",
            "name:=",
            "han",
            " ",
            "tf_prefix:=",
            "",
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # Note: this file includes the "tf_prefix" arg inside it, so that must be defined in this launch file as
    # an argument, even if its not used within the launch file itself.
    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare("netft_utils"), "config", "control_params.yaml"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=node_namespace,
        parameters=[
            robot_description,
            ParameterFile(initial_joint_controllers, allow_substs=True),
        ],
        output="screen",
    )


    # controller_stopper_node = Node(
    #     package="ur_robot_driver",
    #     executable="controller_stopper_node",
    #     namespace=node_namespace,
    #     name="controller_stopper",
    #     output="screen",
    #     emulate_tty=True,
    #     condition=UnlessCondition(use_fake_hardware),
    #     parameters=[
    #         {"headless_mode": False},
    #         {"joint_controller_active": activate_joint_controller},
    #         {
    #             "consistent_controllers": [
    #                 "io_and_status_controller",
    #                 "force_torque_sensor_broadcaster",
    #                 "joint_state_broadcaster",
    #                 "speed_scaling_state_broadcaster",
    #             ]
    #         },
    #     ],
    # )

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


    # # Spawn controllers
    # def controller_spawner(name, active=True):
    #     inactive_flags = ["--inactive"] if not active else []
    #     return Node(
    #         package="controller_manager",
    #         executable="spawner",
    #         namespace=node_namespace,
    #         arguments=[
    #             name,
    #             "--controller-manager",
    #             "controller_manager",
    #             "-n",
    #             node_namespace,
    #             "--controller-manager-timeout",
    #             controller_spawner_timeout,
    #         ]
    #         + inactive_flags,
    #     )

    # controller_spawner_names = [
    #     "joint_state_broadcaster",
    #     "io_and_status_controller",
    #     "speed_scaling_state_broadcaster",
    #     "force_torque_sensor_broadcaster",
    # ]
    # controller_spawner_inactive_names = ["forward_position_controller"]

    # controller_spawners = [
    #     controller_spawner(name) for name in controller_spawner_names
    # ] + [
    #     controller_spawner(name, active=False)
    #     for name in controller_spawner_inactive_names
    # ]

    # # Start scaled_joint_trajectory_controller if real hardware, joint_trajectory_controller otherwise
    # joint_controller_to_start = "joint_trajectory_controller"

    # # There may be other controllers of the joints, but this is the initially-started one
    # initial_joint_controller_spawner_started = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     namespace=node_namespace,
    #     arguments=[
    #         joint_controller_to_start,
    #         "-c",
    #         "controller_manager",
    #         "-n",
    #         node_namespace,
    #         "--controller-manager-timeout",
    #         controller_spawner_timeout,
    #     ],
    #     condition=IfCondition(activate_joint_controller),
    # )
    # initial_joint_controller_spawner_stopped = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     namespace=node_namespace,
    #     arguments=[
    #         joint_controller_to_start,
    #         "-c",
    #         "controller_manager",
    #         "-n",
    #         node_namespace,
    #         "--controller-manager-timeout",
    #         controller_spawner_timeout,
    #         "--inactive",
    #     ],
    #     condition=UnlessCondition(activate_joint_controller),
    # )

    nodes_to_start = [
        control_node,
        ft_broadcaster]
    #     ur_control_node,
    #     dashboard_client_node,
    #     tool_communication_node,
    #     controller_stopper_node,
    #     urscript_interface,
    #     robot_state_publisher_node,
    #     initial_joint_controller_spawner_stopped,
    #     initial_joint_controller_spawner_started,
    # ] + controller_spawners

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "address",
            description="IP address by which the robot can be reached.",
        )
    )
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "tf_prefix",
    #         default_value="han_",
    #         description="tf_prefix of the joint names, useful for \
    #     multi-robot setup. If changed, also joint names in the controllers' configuration \
    #     have to be updated.",
    #     )
    # )
    declared_arguments.append(
        DeclareLaunchArgument(
            "node_namespace",
            default_value="/",
            description="The namespace to put all the nodes into",
        )
    )
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "use_fake_hardware",
    #         default_value="false",
    #         description="Start robot with fake hardware mirroring command to its states.",
    #     )
    # )
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "controller_spawner_timeout",
    #         default_value="10",
    #         description="Timeout used when spawning controllers.",
    #     )
    # )
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "activate_joint_controller",
    #         default_value="true",
    #         description="Activate loaded joint controller.",
    #     )
    # )
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "launch_dashboard_client",
    #         default_value="true",
    #         description="Launch Dashboard Client?",
    #     )
    # )
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "use_tool_communication",
    #         default_value="false",
    #         description="Only available for e series!",
    #     )
    # )
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "tool_device_name",
    #         default_value="/tmp/ttyUR",
    #         description="File descriptor that will be generated for the tool communication device. \
    #         The user has be be allowed to write to this location. \
    #         Only effective, if use_tool_communication is set to True.",
    #     )
    # )
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "tool_tcp_port",
    #         default_value="54321",
    #         description="Remote port that will be used for bridging the tool's serial device. \
    #         Only effective, if use_tool_communication is set to True.",
    #     )
    # )
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
