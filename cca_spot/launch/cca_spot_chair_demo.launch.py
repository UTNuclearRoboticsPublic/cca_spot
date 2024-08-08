import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    ld = LaunchDescription()

    # Define LaunchConfiguration arguments with clear descriptions
    cca_spot_ros_setup = os.path.join(
        get_package_share_directory("cca_spot"), "config", "cca_spot_ros_setup.yaml"
    )

    # Create a Node instance for the cca_spot node
    cc_affordance_planner_ros_node_with_params = Node(
        package="cca_spot",
        executable="cca_spot_chair_demo_node",
        name="cc_affordance_planner_ros",
        emulate_tty=True,
        output="screen",
        prefix=["xterm -e gdb -ex run --args"],
        parameters=[cca_spot_ros_setup],
    )
    ld.add_action(cc_affordance_planner_ros_node_with_params)
    return ld
