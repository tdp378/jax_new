import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    pkg_share = get_package_share_directory('jax_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'jax.urdf.xacro')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': Command(['xacro ', urdf_file])}]
        ),
        # ADD THIS NODE TO GET THE SLIDERS
        #Node(
         #   package='joint_state_publisher_gui',
          #  executable='joint_state_publisher_gui',
           # name='joint_state_publisher_gui'
        #),
     

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        )
    ])
