from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    pkg_capture = FindPackageShare('capture')

    params_path = PathJoinSubstitution([pkg_capture, 'params', 'capture.yaml'])
    # every time the params, rviz, or map path is replaced, the package has to be rebuilt.


    # capture
    node_capture = Node(
        package='capture',
        executable='capture',
        name='capture',
        output='screen',
        parameters=[params_path],
        emulate_tty=True,
    )
    ld.add_action(node_capture)

    # # map_server
    # node_showimage = Node(
    #     package='image_tools',
    #     executable='showimage',
    #     name='showimage',
    #     # output='screen',
    #     remappings=[
    #         ("image", "/camera/image_raw")
    #     ],
    # )
    # ld.add_action(node_showimage)

    return ld