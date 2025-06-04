# Jazzy-Multi-Robot-Sandbox for multi-robot research using ROS Noetic
# Copyright (C) 2025 Alysson Ribeiro da Silva
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

import os
import re
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.conditions import IfCondition
from launch.events import matches_action
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, AndSubstitution, NotSubstitution, PythonExpression as pyexp
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml

def SingleSubstitution(string, token, substitution_obj):
    ss = re.split("(" + token + ")", string)
    cmd_list = []
    for i in range(len(ss)):
        obj = ss[i]

        add = ''
        if i < len(ss)-1:
            add = '+'

        if obj != token:
            obj = "'" + obj + "'" + add
            cmd_list.append(obj)
        else:
            before = "'"
            after = "'" + add
            cmd_list.append(before)
            cmd_list.append(substitution_obj)
            cmd_list.append(after)

    return cmd_list


def generate_launch_description(): 
    ns = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    autostart = LaunchConfiguration('autostart')
    use_lifecycle_manager = LaunchConfiguration("use_lifecycle_manager")
    params_file = LaunchConfiguration('params_file')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='robot_0')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true')
    
    declare_use_lifecycle_manager = DeclareLaunchArgument(
        'use_lifecycle_manager', default_value='false')

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(get_package_share_directory('multi-robot-simulations'), 'config', 'slam.yaml'),
    )

    param_substitutions = {'autostart': 'True',
                    'slam_toolbox.ros_parameters.odom_frame': pyexp(SingleSubstitution('{@}/odom', '{@}', ns)),
                    'slam_toolbox.ros_parameters.map_frame': pyexp(SingleSubstitution('{@}/map', '{@}', ns)),
                    'slam_toolbox.ros_parameters.base_frame': pyexp(SingleSubstitution('{@}/base_link', '{@}', ns)),
                    'slam_toolbox.ros_parameters.scan_topic': pyexp(SingleSubstitution('/{@}/lidar/scan', '{@}', ns))
                    }
    
    configured_slam_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    start_async_slam_toolbox_node = LifecycleNode(
        parameters=[
          configured_slam_params,
          {
            'use_lifecycle_manager': use_lifecycle_manager,
            'use_sim_time': use_sim_time
          }
        ],
        remappings=[("/map", pyexp(SingleSubstitution('/{@}/map', '{@}', ns))),
                    ("/pose", pyexp(SingleSubstitution('/{@}/pose_slam', '{@}', ns)))],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        namespace=ns
    )

    configure_slam_node_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(start_async_slam_toolbox_node),
            transition_id=Transition.TRANSITION_CONFIGURE
        ),
        condition=IfCondition(AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager)))
    )

    activate_slam_node_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=start_async_slam_toolbox_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="[LifecycleLaunch] Slamtoolbox node is activating."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(start_async_slam_toolbox_node),
                    transition_id=Transition.TRANSITION_ACTIVATE
                ))
            ]
        ),
        condition=IfCondition(AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager)))
    )

    return LaunchDescription([
        declare_namespace_cmd,
        declare_params_file,
        declare_autostart_cmd,
        declare_use_lifecycle_manager,
        start_async_slam_toolbox_node,
        configure_slam_node_event,
        activate_slam_node_event
    ])