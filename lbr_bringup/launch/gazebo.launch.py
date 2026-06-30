import os
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _embed_robot_in_world(context, *args, **kwargs):
    """Generate a world SDF with the robot model embedded at launch time.

    Works around gz-sim bugs #3261 / #2957 where self_collide is
    silently ignored on URDF models spawned via ``ros_gz_sim create``.
    Embedding the model directly in the world file forces Gazebo to
    parse self_collide during world loading, when it is respected.
    """
    robot_name = LaunchConfiguration("robot_name").perform(context)

    # ── Run xacro to get the robot URDF ──────────────────────────────────
    xacro_cmd = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathSubstitution(
                FindPackageShare("lbr_description")
            )
            / "urdf"
            / LaunchConfiguration("model")
            / LaunchConfiguration("model"),
            ".xacro",
            " robot_name:=",
            LaunchConfiguration("robot_name"),
            " mode:=gazebo",
            " initial_joint_positions_path:=",
            PathSubstitution(
                FindPackageShare(
                    LaunchConfiguration("init_jnt_pos_pkg")
                )
            )
            / LaunchConfiguration("init_jnt_pos"),
        ]
    )
    robot_urdf = context.perform_substitution(xacro_cmd)

    # ── Write URDF to a temp file so Gazebo can <include> it ────────────
    tmp_dir = tempfile.mkdtemp(prefix="gz_world_")
    urdf_path = os.path.join(tmp_dir, "robot.urdf")
    with open(urdf_path, "w", encoding="utf-8") as f:
        f.write(robot_urdf)

    # ── Generate world SDF with the robot embedded ──────────────────────
    world_sdf = f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="empty">
    <physics name="1ms" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics">
    </plugin>
    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands">
    </plugin>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster">
    </plugin>

    <include>
      <uri>file://{urdf_path}</uri>
      <name>{robot_name}</name>
      <pose>0 0 0 0 0 0</pose>
    </include>
  </world>
</sdf>"""

    world_path = os.path.join(tmp_dir, "world.sdf")
    with open(world_path, "w", encoding="utf-8") as f:
        f.write(world_sdf)

    # ── Return the Gazebo launch action with the generated world ────────
    return [
        IncludeLaunchDescription(
            PathSubstitution(
                FindPackageShare("ros_gz_sim"),
            )
            / "launch"
            / "gz_sim.launch.py",
            launch_arguments={"gz_args": f"-r {world_path}"}.items(),
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="model",
                default_value="iiwa7",
                description="The LBR model in use.",
                choices=["iiwa7", "iiwa14", "med7", "med14"],
            ),
            DeclareLaunchArgument(
                name="robot_name",
                default_value="lbr",
                description=(
                    "The robot's name. Links in the tf tree will be prefixed as "
                    "<robot_name>_link. Same applies to joints. "
                    "The robot's name will be used as namespace."
                ),
            ),
            DeclareLaunchArgument(
                name="init_jnt_pos_pkg",
                default_value="lbr_description",
                description="Package containing the initial_joint_positions.yaml file.",
            ),
            DeclareLaunchArgument(
                name="init_jnt_pos",
                default_value="ros2_control/initial_joint_positions.yaml",
                description=(
                    "The relative path from sys_cfg_pkg to the "
                    "initial_joint_positions.yaml file."
                ),
            ),
            DeclareLaunchArgument(
                name="ctrl",
                default_value="joint_trajectory_controller",
                description=(
                    "Desired default controller. Gazebo loads controller "
                    "configuration through lbr_description/gazebo/*.xacro from "
                    "lbr_description/ros2_control/gazebo_controllers.yaml."
                ),
                choices=[
                    "forward_position_controller",
                    "joint_trajectory_controller",
                ],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": ParameterValue(
                            Command(
                                [
                                    FindExecutable(name="xacro"),
                                    " ",
                                    PathSubstitution(
                                        FindPackageShare("lbr_description")
                                    )
                                    / "urdf"
                                    / LaunchConfiguration("model")
                                    / LaunchConfiguration("model"),
                                    ".xacro",
                                    " robot_name:=",
                                    LaunchConfiguration("robot_name"),
                                    " mode:=gazebo",
                                    " initial_joint_positions_path:=",
                                    PathSubstitution(
                                        FindPackageShare(
                                            LaunchConfiguration("init_jnt_pos_pkg")
                                        )
                                    )
                                    / LaunchConfiguration("init_jnt_pos"),
                                ]
                            ),
                            value_type=str,
                        )
                    },
                    {"use_sim_time": True},
                ],
                namespace=LaunchConfiguration("robot_name"),
            ),
            # ── Gazebo with robot embedded in the world SDF ──
            # This replaces the old IncludeLaunchDescription(gz_sim) +
            # ros_gz_sim create pattern.  The robot is included via
            # <include> in a generated world file so that self_collide
            # is respected (gz-sim #3261, #2957).
            OpaqueFunction(function=_embed_robot_in_world),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=[
                    "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                    "/ft_sensor/wrench@geometry_msgs/msg/WrenchStamped[gz.msgs.Wrench",
                ],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                output="screen",
                arguments=[
                    "--controller-manager",
                    "controller_manager",
                    "joint_state_broadcaster",
                    LaunchConfiguration("ctrl"),
                ],
                namespace=LaunchConfiguration("robot_name"),
            ),
        ]
    )
