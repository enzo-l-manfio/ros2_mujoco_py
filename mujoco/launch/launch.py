import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    directory = get_package_share_directory('robot_description')
    xmlPath = os.path.join(directory, 'model', 'sphere.xml')

    mujoco = Node(
        package = "mujoco",
        executable="mujoco_node",
        output="screen",
        parameters=[
            {"xmlPath": xmlPath},
            {"dt": 0.00097}
        ]
    )

    return LaunchDescription([mujoco])