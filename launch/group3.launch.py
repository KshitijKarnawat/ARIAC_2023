from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare

from ariac_moveit_config.parameters import generate_parameters

def generate_launch_description():

    # Robot Commander Node
    robot_commander = Node(
        package="group3",
        executable="group3_exe",
        output="screen",
        parameters=generate_parameters()
    )

    part_detector_start = Node(
        package="group3",
        executable="part_detector.py",
        output="screen",
    )

    # floor_commander = Node(
    #     package="group3",
    #     executable="floor_exe",
    #     output="screen",
    #     parameters=generate_parameters()
    # )

    # MoveIt node
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ariac_moveit_config"), "/launch", "/ariac_robots_moveit.launch.py"]
        )
    )
    # return LaunchDescription([robot_commander, floor_commander, moveit])
    return LaunchDescription([robot_commander,part_detector_start,moveit])