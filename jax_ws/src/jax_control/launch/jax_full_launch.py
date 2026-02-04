from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # --- 1. CLEANUP ---
    # This kills old instances so you don't end up with multiple 
    # controller logs, RViz windows, or Teleop tabs.
    os.system('pkill -f rviz2')
    os.system('pkill -f jax_controller')
    os.system('pkill -f teleop_twist_keyboard')

    # --- 2. NODES ---
    # The Controller Node
    controller_node = Node(
        package='jax_control',
        executable='jax_controller',
        name='jax_controller',
        output='screen'
    )

    # RViz (Including the launch file from jax_description)
    description_share = get_package_share_directory('jax_description')
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_share, 'launch', 'display.launch.py')
        )
    )

    # --- 3. TELEOP (LAUNCHED IN A TAB) ---
    # Using 'pkill' above ensures we don't double up.
    # gnome-terminal will now open this as a tab in your existing window.
    teleop_node = ExecuteProcess(
        cmd=['gnome-terminal', '--tab', '--title=JAX_TELEOP', '--', 'ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard'],
        output='screen'
    )

    return LaunchDescription([
        controller_node,
        rviz_launch,
        teleop_node
    ])