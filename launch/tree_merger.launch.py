
import launch

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():

    parent_link = DeclareLaunchArgument(
        'parent_link',
        description="Name of the link of the main tf tree where child_link should be attached")

    child_link = DeclareLaunchArgument(
        'child_link',
        description="Name of the child link")

    child_link_root = DeclareLaunchArgument(
        'child_link_root',
        description="Name of the root link of the tf tree to which child_link belongs")

    child_to_parent_tf = DeclareLaunchArgument(
        'child_to_parent_tf',
        default_value="0 0 0 0 0 0 1",
        description="Transform between child_link -> parent_link"
    )

    static_merger_node = Node(
            package='static_tree_merger',
            executable='static_tree_merger',
            name='static_tree_merger',
            namespace="kinect",
            output='screen',
            parameters=[
                {'parent_link': launch.substitutions.LaunchConfiguration('parent_link')},
                {'child_link': launch.substitutions.LaunchConfiguration('child_link')},
                {'child_link_root': launch.substitutions.LaunchConfiguration('child_link_root')},
                {'child_to_parent_tf': launch.substitutions.LaunchConfiguration('child_to_parent_tf')},
            ]
        )


    

    ll = list()
    ll.append(parent_link)
    ll.append(child_link)
    ll.append(child_link_root)
    ll.append(child_to_parent_tf)
    ll.append(static_merger_node)

    return LaunchDescription(ll)