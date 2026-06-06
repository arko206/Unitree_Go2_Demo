from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = FindPackageShare('go2_local_description')
    default_urdf = PathJoinSubstitution([pkg_share, 'urdf', 'go2_description.urdf'])

    urdf_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value=default_urdf,
        description='Path to robot URDF or Xacro within this package'
    )

    urdf_file = LaunchConfiguration('urdf_file')
    xacro_cmd = Command([FindExecutable(name='xacro'), ' ', urdf_file])
    robot_description = ParameterValue(xacro_cmd, value_type=str)

    return LaunchDescription([
        urdf_arg,
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2'
        ),
    ])
