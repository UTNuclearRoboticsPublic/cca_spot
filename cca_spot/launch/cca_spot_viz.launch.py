"""
Author: Crasun Jans

Description:
This launch script launches the `cca_ros_viz` node with the robot's visualization-related parameters along with the URDF and SRDF data passed as robot_description and robot_description_semantic parameters respectively. It also launches RViz with a predefined configuration file for visualizing the robot.

### Overview:
The script generates three key parameters:
1. `robot_description`: Contains the robot's URDF, defining its physical structure, sensors, and actuators.
2. `robot_description_semantic`: Contains the robot's SRDF, defining its semantic properties, such as joint groups and kinematics for motion planning.
3. `cca_ros_viz_setup_params`: Contains parameters such as joint_states_topic, planning_group, etc. specified in the cca_<robot>_ros_viz_setup.yaml file in the cca_<robot> package.

### Customization:
To adapt this script for a different robot, modify only the following functions:
- `generate_robot_description_content()`: Adjust how the URDF content is extracted for your robot.
- `generate_robot_description_semantic_content()`: Update this function to provide the correct SRDF file for the robot.
- `extract_cca_ros_viz_setup_params()`: Update this function to provide the correct path info to the cca_<robot>_ros_viz_setup.yaml file.
"""
import os
from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable
from launch_ros.descriptions import ParameterValue


def generate_robot_description_content():
    """
    Generates the robot_description_content, which contains the robot's URDF.

    Returns:
        - robot_description_content: The robot description generated from the xacro file.
        - launch_args: Launch arguments needed to extract the robot description.
    """
    launch_args = [
        DeclareLaunchArgument(
            'has_arm',
            description='Boolean. Include the Spot Arm.',
            choices=['True', 'False'],
            default_value='True'
        ),
        DeclareLaunchArgument(
            'has_eap',
            description='Boolean. Include the Enhanced Autonomy package (EAP)',
            choices=['True', 'False'],
            default_value='False'
        ),
        DeclareLaunchArgument(
            'has_eap_2',
            description='Boolean. Include the Updated Enhanced Autonomy package (EAP2)',
            choices=['True', 'False'],
            default_value='False'
        ),
        DeclareLaunchArgument(
            'has_realsense',
            description='Boolean. Include an arm-mounted Realsense D435',
            choices=['True', 'False'],
            default_value='False'
        ),
        DeclareLaunchArgument(
            'has_cam_payload',
            description='Boolean. Include the CAM payload',
            choices=['True', 'False'],
            default_value='False'
        ),
    ]

    has_arm = LaunchConfiguration('has_arm')
    has_eap = LaunchConfiguration('has_eap')
    has_eap_2 = LaunchConfiguration('has_eap_2')
    has_realsense = LaunchConfiguration('has_realsense')
    has_cam_payload = LaunchConfiguration('has_cam_payload')

    # Path to the robot xacro file
    this_pkg_share = FindPackageShare('spot_description')
    xacro_path = PathJoinSubstitution([this_pkg_share, 'urdf', 'spot.urdf.xacro'])

    # Generate the robot description by processing the xacro file with the specified configurations
    robot_description_content = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            ' ',
            xacro_path,
            ' has_arm:=', has_arm,
            ' has_eap:=', has_eap,
            ' has_eap_2:=', has_eap_2,
            ' has_realsense:=', has_realsense,
            ' has_cam_payload:=', has_cam_payload
        ]),
        value_type=str
    )

    return robot_description_content, launch_args


def generate_robot_description_semantic_content():
    """
    Generates the robot_description_semantic_content, which contains the SRDF (semantic robot description).
    This is typically used for defining robot groups, end-effectors, and kinematics configurations.

    Returns:
        - robot_description_semantic_content: The semantic robot description loaded from the SRDF file.
    """
    # Path to the robot SRDF file
    robot_description_semantic_path = os.path.join(
        get_package_share_directory('spot_arm_moveit2_config'),
        'config',
        'spot.srdf'
    )

    # Load the SRDF content from the file
    try:
        with open(robot_description_semantic_path, 'r') as srdf_file:
            robot_description_semantic_content = srdf_file.read()
    except FileNotFoundError:
        raise RuntimeError(f"SRDF file not found: {robot_description_semantic_path}")

    return robot_description_semantic_content

def extract_cca_ros_viz_setup_params():
    """
    Extracts cca-visualization-related parameters for the robot from the specified yaml file

    Returns:
        - cca_ros_viz_setup_params: List of parameters such as joint_states_topic, planning_group, etc. specified in the cca_<robot>_ros_viz_setup.yaml file in the cca_<robot> package
    """

    cca_ros_viz_setup_params = os.path.join(
        get_package_share_directory('cca_spot'),
        'config',
        'cca_spot_ros_viz_setup.yaml'
        )

    return cca_ros_viz_setup_params


def generate_launch_description():
    """
    Generates the full launch description to launch the cca_ros_viz node with robot_description, robot_description_semantic parameters, and RViz with a custom configuration.
    """
    launch_args = [DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulated time (for simulation environments)'
    )]

    robot_description_content, description_launch_args = generate_robot_description_content()
    robot_description_semantic_content = generate_robot_description_semantic_content()
    cca_ros_viz_setup_params = extract_cca_ros_viz_setup_params()

    robot_description = {"robot_description": robot_description_content}
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}

    use_sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}

    # Default Rviz config file
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('cca_ros_viz'),
        'rviz',
        'cca_ros_viz.rviz'
    ])
    print(rviz_config_file)

    # Return the launch description, including robot description-related arguments and RViz node
    return launch.LaunchDescription(
        description_launch_args + launch_args + [
            # Launch cca_ros_viz node
            launch_ros.actions.Node(
                package='cca_ros_viz',
                executable='cca_ros_viz_node',
                name='cca_ros_viz',
                output='screen',
                parameters=[
                    robot_description,
                    robot_description_semantic,
                    cca_ros_viz_setup_params,
                    use_sim_time
                ],
            ),
            # Launch RViz with the specified configuration
            launch_ros.actions.Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config_file],  # Load RViz config file
                parameters=[
                    robot_description,
                    robot_description_semantic,
                    use_sim_time
                ],
            ),
        ]
    )
