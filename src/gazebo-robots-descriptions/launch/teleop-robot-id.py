#import sys
#import os

#from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

from launch_ros.actions import Node as create_node_description
#from launch.launch_description_sources import PythonLaunchDescriptionSource as load_python_launch_file
#from launch.actions import IncludeLaunchDescription as include_another_launch_file 
from launch.substitutions import LaunchConfiguration as get_set_argument_val
#from launch.substitutions import PathJoinSubstitution as concatenate_path_strings
#from launch.substitutions import TextSubstitution as replace_string
from launch.actions import DeclareLaunchArgument as create_input_argument

def generate_launch_description(): 
    robot_namespace = create_input_argument('namespace', default_value='robot_0')
    get_set_argument_val('namespace')

    teleop_node = create_node_description(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop_twist_keyboard",
        namespace=get_set_argument_val('namespace'),
        # remappings=[('cmd_vel', concatenate_path_strings([get_set_argument_val('namespace'), "cmd_vel"]))],
        prefix = 'xterm -e',
        output='screen'
    )

    return LaunchDescription([robot_namespace, teleop_node])

