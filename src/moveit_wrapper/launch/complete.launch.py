from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = []
    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="10.181.116.103",
            description="IP address by which the robot can be reached."
        )
    )
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="ur_bringup",
            description='Package with the controller\'s configuration in "config" folder. \
            Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="ur_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
            multi-robot setup. If changed than also joint names in the controllers' configuration \
            have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_controller",
            default_value="joint_trajectory_controller",
            description="Robot controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_dashboard_client", default_value="false", description="Launch RViz?"
        )
    )

    robot_ip = LaunchConfiguration("robot_ip")
    # General arguments
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    prefix = LaunchConfiguration("prefix")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    robot_controller = LaunchConfiguration("robot_controller")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_dashboard_client = LaunchConfiguration("launch_dashboard_client")

    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('cc_ur_config'), 'launch', 'ur_control.launch.py'])),
        launch_arguments={
            "robot_ip": robot_ip,
            "runtime_config_package": runtime_config_package,
            "controllers_file": controllers_file,
            "prefix": prefix,
            "use_fake_hardware": use_fake_hardware,
            "robot_controller": robot_controller,
            "launch_dashboard_client": launch_dashboard_client
        }.items(),
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('cc_ur_config'), 'launch', 'ur_moveit.launch.py'])),
        launch_arguments={
            "robot_ip": robot_ip,
            "prefix": prefix,
            "use_fake_hardware": use_fake_hardware,
            "launch_rviz": launch_rviz
        }.items(),
    )

    moveit_wrapper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('moveit_wrapper'), 'launch', 'moveit_wrapper.launch.py'])),
        launch_arguments={
            "robot_ip": robot_ip,
            "prefix": prefix,
            "use_fake_hardware": use_fake_hardware,
        }.items(),
    )
    return LaunchDescription(declared_arguments + [control_launch, moveit_launch, moveit_wrapper_launch])
