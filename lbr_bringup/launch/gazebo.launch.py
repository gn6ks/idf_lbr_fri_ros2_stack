import os
import subprocess
import tempfile
import xml.etree.ElementTree as ET

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
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
    """Generate a world SDF with the robot model directly embedded.

    Works around gz-sim bugs #3261 / #2957 where self_collide is
    silently ignored on URDF models spawned via ``ros_gz_sim create``
    or included via ``<include>`` at world parse time.

    The URDF is converted to SDF with ``gz sdf -p`` and the resulting
    ``<model>`` element is injected straight into the world SDF — no
    ``<include>``, no runtime spawn.  This is the only path where
    Gazebo Harmonic reliably respects ``self_collide``.
    """
    robot_name = LaunchConfiguration("robot_name").perform(context)

    # ── 1. Run xacro to get the robot URDF ──────────────────────────
    xacro_cmd = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathSubstitution(FindPackageShare("lbr_description"))
            / "urdf"
            / LaunchConfiguration("model")
            / LaunchConfiguration("model"),
            ".xacro",
            " robot_name:=",
            LaunchConfiguration("robot_name"),
            " mode:=gazebo",
            " initial_joint_positions_path:=",
            PathSubstitution(FindPackageShare(LaunchConfiguration("init_jnt_pos_pkg")))
            / LaunchConfiguration("init_jnt_pos"),
        ]
    )
    robot_urdf = context.perform_substitution(xacro_cmd)

    # ── 2. Write URDF to temp file ───────────────────────────────────
    tmp_dir = tempfile.mkdtemp(prefix="gz_world_")
    urdf_path = os.path.join(tmp_dir, "robot.urdf")
    with open(urdf_path, "w", encoding="utf-8") as f:
        f.write(robot_urdf)

    # ── 3. Convert URDF → SDF with gz sdf -p ───────────────────────
    try:
        result = subprocess.run(
            ["gz", "sdf", "-p", urdf_path],
            capture_output=True,
            text=True,
            timeout=15,
        )
        sdf_text = result.stdout
    except (FileNotFoundError, subprocess.TimeoutExpired):
        # Fallback: embed the URDF as a model inside an SDF world.
        # gz-sim can parse URDF when it is placed directly under
        # <world> as an <include> with merge=true.
        sdf_text = None

    # ── 4. Extract the <model> block from the SDF output ────────────
    model_block = None
    if sdf_text:
        try:
            root = ET.fromstring(sdf_text)
            # gz sdf -p outputs <sdf><model name="...">...</model></sdf>
            model_el = root.find("model")
            if model_el is not None:
                # Force <self_collide>false</self_collide> even if the
                # converter missed it (belt-and-suspenders).
                sc = model_el.find("self_collide")
                if sc is None:
                    sc = ET.SubElement(model_el, "self_collide")
                sc.text = "false"
                # Rename the model to match the robot_name launch arg
                model_el.set("name", robot_name)
                model_block = ET.tostring(model_el, encoding="unicode")
        except ET.ParseError:
            pass

    # ── 5. Fallback: if gz sdf failed, inline the URDF via <include> ─
    if model_block is None:
        model_block = (
            f"<include>"
            f"<uri>file://{urdf_path}</uri>"
            f"<name>{robot_name}</name>"
            f"<pose>0 0 0 0 0 0</pose>"
            f"</include>"
        )

    # ── 6. Generate the world SDF with the robot embedded AND a     ──
    #      separate "ghost" screen model for cross-model collision.  ──
    #      Workaround for gz-sim #3261 / #2957: self_collide is      ──
    #      ignored on spawned/included URDF models, but cross-model   ──
    #      collision between SEPARATE models always works.            ──
    #      By placing the screen as an independent static model,      ──
    #      the sponge-tool will collide with it regardless of the     ──
    #      robot model's broken self_collide.                         ──

    # Screen position in world coordinates (matches iiwa7.xacro):
    #   lbr_base_link origin:  0, 0, 0.660  (lbr_world_base_joint)
    #   screen collision:     -0.2195, 0, 0.1615  (relative to base_link)
    #   → world:              -0.2195, 0, 0.8215
    screen_x = -0.2195
    screen_y = 0.0
    screen_z = 0.8215
    screen_box = "0.389 0.705 0.323"

    # Ghost screen: separate static model so physics ALWAYS checks
    # collision against the robot tool chain (cross-model).
    ghost_screen = f"""
    <model name="screen_collision" canonical="true">
      <static>true</static>
      <pose>{screen_x} {screen_y} {screen_z} 0 0 0</pose>
      <link name="screen_link">
        <collision name="screen_collision">
          <geometry>
            <box>
              <size>{screen_box}</size>
            </box>
          </geometry>
          <surface>
            <contact>
              <ode>
                <kp>100000.0</kp>
                <kd>100.0</kd>
                <max_vel>0.01</max_vel>
                <min_depth>0.001</min_depth>
              </ode>
            </contact>
            <friction>
              <ode>
                <mu>0.5</mu>
                <mu2>0.3</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
      </link>
    </model>
"""

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

{model_block}
{ghost_screen}
  </world>
</sdf>"""

    world_path = os.path.join(tmp_dir, "world.sdf")
    with open(world_path, "w", encoding="utf-8") as f:
        f.write(world_sdf)

    # ── 7. Return Gazebo launch action ─────────────────────────────
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
