from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import ExecuteProcess


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

  

    unpause = ExecuteProcess(
    cmd=[
        "ros2", "service", "call",
        "/world/empty/control",
        "ros_gz_interfaces/srv/ControlWorld",
        "{world_control: {pause: false}}",
    ],
    output="screen",
)


        # Disable gravity
    disable_gravity = ExecuteProcess(
        cmd=[
            "ros2", "service", "call",
            "/world/empty/set_physics",
            "ros_gz_interfaces/srv/SetPhysics",
            "{physics: {gravity: {x: 0.0, y: 0.0, z: 0.0}}}",
        ],
        output="screen",
    )

    # Re-enable gravity
    enable_gravity = ExecuteProcess(
        cmd=[
            "ros2", "service", "call",
            "/world/empty/set_physics",
            "ros_gz_interfaces/srv/SetPhysics",
            "{physics: {gravity: {x: 0.0, y: 0.0, z: -9.81}}}",
        ],
        output="screen",
    )


    # Package paths
    desc_pkg_share = get_package_share_directory("jax_description")
    bringup_pkg_share = get_package_share_directory("jax_bringup")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    xacro_file = os.path.join(desc_pkg_share, "urdf", "jax.urdf.xacro")
    controllers_file = os.path.join(bringup_pkg_share, "config", "jax_controllers.yaml")
    jax_share = desc_pkg_share  # used by xacro for absolute mesh paths

    # robot_description must be a STRING (ParameterValue fix)
    robot_description = ParameterValue(
        Command([
            "xacro ",
            xacro_file,
            " ",
            "controllers_yaml:=",
            controllers_file,
            " ",
            "jax_share:=",
            TextSubstitution(text=jax_share),
        ]),
        value_type=str,
    )

    # Gazebo Sim (RUNNING)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": "empty.sdf -r"}.items(),
    )

    # Robot State Publisher
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"robot_description": robot_description},
        ],
        output="screen",
    )

    # Spawn robot into Gazebo (above ground)
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", "jax",
            "-z", "0.25",
        ],
        output="screen",
    )

    # Controllers
    spawn_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )

    spawn_legs = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "legs_controller",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )

    # Initial pose publisher (you must have console_script: initial_pose)
    initial_pose_node = Node(
        package="jax_bringup",
        executable="initial_pose",
        output="screen",
       
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),

        gazebo,
        rsp,

        TimerAction(period=1.0, actions=[spawn]),
        TimerAction(period=4.0, actions=[spawn_jsb]),
        TimerAction(period=5.0, actions=[spawn_legs]),
        TimerAction(period=5.3, actions=[initial_pose_node]),


    ])
