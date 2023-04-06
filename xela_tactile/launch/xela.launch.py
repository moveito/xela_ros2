import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():
    parameter_file = load_yaml("xela_tactile", "config/parameter.yaml")

    container = ComposableNodeContainer(
        name="xela_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="xela_tactile",
                plugin="xela_tactile::XelaTactile",
                name="xela_tactile",
                parameters=[
                    parameter_file
                ],
            ),
        ],
        output="screen",
    )

    ld = LaunchDescription()

    ld.add_action(container)

    return ld
