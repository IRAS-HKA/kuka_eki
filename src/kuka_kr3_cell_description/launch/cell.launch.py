from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="10.181.116.41",
            description="IP address by which the robot can be reached.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(DeclareLaunchArgument(
        "eki_robot_port",
        default_value="54600",
        description="Port by which the robot can be reached."
    ))

    declared_arguments.append(DeclareLaunchArgument(
        "eki_io_port",
        default_value="54601",
        description="Port by which the robot can be reached."
    ))

    declared_arguments.append(DeclareLaunchArgument(
        "n_io",
        default_value="2",
        description="Port by which the robot can be reached."
    ))

    # Initialize Arguments
    robot_ip = LaunchConfiguration("robot_ip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    eki_robot_port = LaunchConfiguration("eki_robot_port")
    eki_io_port = LaunchConfiguration("eki_io_port")
    n_io = LaunchConfiguration("n_io")

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('kuka_common_moveit_config'), 'launch', 'demo.launch.py'])),
        launch_arguments={
            "robot_ip": robot_ip,
            "use_fake_hardware": use_fake_hardware,
            "eki_robot_port": eki_robot_port,
            "robot_description_package": 'kuka_kr3_cell_description'
        }.items(),
    )

    gripper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('zimmer_gp406n'), 'launch', 'zimmer_gp406_node.launch.py'])),
        launch_arguments={
            "robot_ip": robot_ip,
            "eki_io_port": eki_io_port,
            "n_io": n_io,
        }.items(),
    )

    moveit_wrapper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('moveit_wrapper'), 'launch', 'moveit_wrapper.launch.py'])),
        launch_arguments={
            "robot_ip": robot_ip,
            "use_fake_hardware": use_fake_hardware,
            "eki_robot_port": eki_robot_port,
            "robot_description_package": 'kuka_kr3_cell_description'
        }.items(),
    )

    return LaunchDescription(declared_arguments + [moveit_launch, gripper_launch, moveit_wrapper_launch])
