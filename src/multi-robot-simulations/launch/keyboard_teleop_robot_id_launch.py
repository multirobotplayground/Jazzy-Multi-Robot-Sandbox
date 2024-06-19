# Jazzy-Multi-Robot-Sandbox for multi-robot research using ROS Noetic
# Copyright (C) 2024 Alysson Ribeiro da Silva
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

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
    robot_namespace = create_input_argument('namespace', default_value='')
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

