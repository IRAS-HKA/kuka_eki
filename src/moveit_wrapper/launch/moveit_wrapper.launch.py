import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument(
        "robot_description_package",
        default_value="kuka_kr3_cell_description",
        description="Robot description package.",
    ))

    declared_arguments.append(DeclareLaunchArgument(
        "robot_description_file",
        default_value="robot.urdf.xacro",
        description="Robot description file located in <robot_description_package>/urdf/ .",
    ))

    declared_arguments.append(DeclareLaunchArgument(
        "semantic_description_file",
        default_value="robot.srdf",
        description="Semantic robot description file located in <robot_description_package>/config/ .",
    ))

    declared_arguments.append(DeclareLaunchArgument(
        "use_fake_hardware",
        default_value="true",
        description="Start robot with fake hardware mirroring command to its states.",
    ))

    declared_arguments.append(DeclareLaunchArgument(
        "robot_ip",
        default_value="10.181.116.41",
        description="IP address by which the robot can be reached."
    ))

    declared_arguments.append(DeclareLaunchArgument(
        "eki_robot_port",
        default_value="54600",
        description="Port by which the robot can be reached."
    ))

    robot_description_package = LaunchConfiguration("robot_description_package")
    robot_description_file = LaunchConfiguration("robot_description_file")
    semantic_description_file = LaunchConfiguration("semantic_description_file")
    robot_ip = LaunchConfiguration("robot_ip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    eki_robot_port = LaunchConfiguration("eki_robot_port")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(robot_description_package), "urdf", robot_description_file]),
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
            " ",
            "robot_ip:=",
            robot_ip,
            " ",
            "eki_robot_port:=",
            eki_robot_port,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(robot_description_package), "config", semantic_description_file]
            ),
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }

    kinematics_yaml = load_yaml(
        "kuka_common_moveit_config", "config/kinematics.yaml"
    )

    planning_group = {"planning_group": "manipulator"}

    moveit_wrapper = Node(
        package="moveit_wrapper",
        executable="moveit_wrapper_node",
        output="screen",
        parameters=[robot_description, robot_description_semantic, kinematics_yaml, planning_group],
    )

    return LaunchDescription(declared_arguments + [moveit_wrapper])
