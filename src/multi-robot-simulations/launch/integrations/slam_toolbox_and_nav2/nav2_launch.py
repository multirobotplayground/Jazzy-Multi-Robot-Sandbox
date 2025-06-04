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
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, PythonExpression as pyexp
from launch_ros.actions import SetParameter, Node
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
    bringup_dir = get_package_share_directory('multi-robot-simulations')
    
    ns = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'velocity_smoother',
        'collision_monitor',
        'bt_navigator',
        'waypoint_follower',
        'docking_server',
    ]

    param_substitutions = {'autostart': 'True',
                    'amcl.ros_parameters.base_frame_id': pyexp(SingleSubstitution('{@}/base_link', '{@}', ns)),
                    'amcl.ros_parameters.global_frame_id': pyexp(SingleSubstitution('{@}/map', '{@}', ns)),
                    'amcl.ros_parameters.odom_frame_id': pyexp(SingleSubstitution('{@}/odom', '{@}', ns)),
                    'amcl.ros_parameters.scan_topic': pyexp(SingleSubstitution('/{@}/lidar/scan', '{@}', ns)),
                    'bt_navigator.ros_parameters.global_frame': pyexp(SingleSubstitution('{@}/map', '{@}', ns)),
                    'bt_navigator.ros_parameters.robot_base_frame': pyexp(SingleSubstitution('{@}/base_link', '{@}', ns)),
                    'bt_navigator.ros_parameters.odom_topic': pyexp(SingleSubstitution('{@}/odom', '{@}', ns)),
                    'local_costmap.local_costmap.ros_parameters.global_frame': pyexp(SingleSubstitution('{@}/odom', '{@}', ns)),
                    'local_costmap.local_costmap.ros_parameters.robot_base_frame': pyexp(SingleSubstitution('{@}/base_link', '{@}', ns)),
                    'local_costmap.local_costmap.ros_parameters.voxel_layer.scan.topic': pyexp(SingleSubstitution('/{@}/lidar/scan', '{@}', ns)),
                    'global_costmap.global_costmap.ros_parameters.global_frame': pyexp(SingleSubstitution('{@}/map', '{@}', ns)),
                    'global_costmap.global_costmap.ros_parameters.robot_base_frame': pyexp(SingleSubstitution('{@}/base_link', '{@}', ns)),
                    'global_costmap.global_costmap.ros_parameters.obstacle_layer.scan.topic': pyexp(SingleSubstitution('/{@}/lidar/scan', '{@}', ns)),
                    'behavior_server.ros_parameters.local_costmap_topic': pyexp(SingleSubstitution('/{@}/local_costmap/costmap_raw', '{@}', ns)),
                    'behavior_server.ros_parameters.global_costmap_topic': pyexp(SingleSubstitution('/{@}/global_costmap/costmap_raw', '{@}', ns)),
                    'behavior_server.ros_parameters.local_footprint_topic': pyexp(SingleSubstitution('/{@}/global_costmap/published_footprint', '{@}', ns)),
                    'behavior_server.ros_parameters.local_frame': pyexp(SingleSubstitution('{@}/odom', '{@}', ns)),
                    'behavior_server.ros_parameters.global_frame': pyexp(SingleSubstitution('{@}/map', '{@}', ns)),
                    'behavior_server.ros_parameters.robot_base_frame': pyexp(SingleSubstitution('{@}/base_link', '{@}', ns)),
                    'velocity_smoother.ros_parameters.odom_topic': pyexp(SingleSubstitution('/{@}/odom', '{@}', ns)),
                    'collision_monitor.ros__parameters.base_frame_id': pyexp(SingleSubstitution('{@}/base_link', '{@}', ns)),
                    'collision_monitor.ros__parameters.odom_frame_id': pyexp(SingleSubstitution('{@}/odom', '{@}', ns)),
                    'collision_monitor.ros__parameters.cmd_vel_in_topic': pyexp(SingleSubstitution('/{@}/cmd_vel_nav', '{@}', ns)),
                    'collision_monitor.ros__parameters.cmd_vel_out_topic': pyexp(SingleSubstitution('/{@}/cmd_vel', '{@}', ns)),
                    'collision_monitor.ros__parameters.state_topic': pyexp(SingleSubstitution('/{@}/collision_monitor_state', '{@}', ns)),
                    'collision_monitor.ros__parameters.FootprintApproach.footprint_topic': pyexp(SingleSubstitution('/{@}/local_costmap/published_footpprint', '{@}', ns)),
                    'collision_monitor.ros__parameters.FootprintApproach.scan.topic': pyexp(SingleSubstitution('/{@}/lidar/scan', '{@}', ns)),
                    'docking_server.ros__parameters.base_frame': pyexp(SingleSubstitution('{@}/base_link', '{@}', ns)),
                    'docking_server.ros__parameters.fixed_frame': pyexp(SingleSubstitution('{@}/odom', '{@}', ns)),
                    'docking_server.ros__parameters.controller.costmap_topic': pyexp(SingleSubstitution('/{@}/local_costmap/costmap_raw', '{@}', ns)),
                    'docking_server.ros__parameters.controller.footprint_topic': pyexp(SingleSubstitution('/{@}/local_costmap/published_footprint', '{@}', ns)),
                    'loopback_simulator.ros__parameters.base_frame_id': pyexp(SingleSubstitution('{@}/base_link', '{@}', ns)),
                    'loopback_simulator.ros__parameters.odom_frame_id': pyexp(SingleSubstitution('{@}/odom', '{@}', ns)),
                    'loopback_simulator.ros__parameters.map_frame_id': pyexp(SingleSubstitution('{@}/map', '{@}', ns)),
                    'loopback_simulator.ros__parameters.scan_frame_id': pyexp(SingleSubstitution('{@}/base_link', '{@}', ns)),
                    'slam_toolbox.ros_parameters.odom_frame': pyexp(SingleSubstitution('{@}/odom', '{@}', ns)),
                    'slam_toolbox.ros_parameters.map_frame': pyexp(SingleSubstitution('{@}/map', '{@}', ns)),
                    'slam_toolbox.ros_parameters.base_frame': pyexp(SingleSubstitution('{@}/base_link', '{@}', ns)),
                    'slam_toolbox.ros_parameters.scan_topic': pyexp(SingleSubstitution('/{@}/lidar/scan', '{@}', ns))
                    }

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=ns,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'config', 'nav2_common.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack',
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='False',
        description='Use composed bringup if True',
    )

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name',
        default_value='nav2_container',
        description='the name of conatiner that nodes will load in if use composition',
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.',
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='log level'
    )

    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                respawn=use_respawn,
                namespace=ns,
                respawn_delay=2.0,
                parameters=[configured_params,
                            {'use_sim_time': True}],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=[(pyexp(SingleSubstitution('/{@}/tf', '{@}', ns)), '/tf'), 
                            (pyexp(SingleSubstitution('/{@}/tf_static', '{@}', ns)), 'tf_static'),
                            ('/cmd_vel', pyexp(SingleSubstitution('/{@}/cmd_vel_nav', '{@}', ns)))]
            ),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                namespace=ns,
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=[(pyexp(SingleSubstitution('/{@}/tf', '{@}', ns)), '/tf'), 
                            (pyexp(SingleSubstitution('/{@}/tf_static', '{@}', ns)), 'tf_static')],
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=use_respawn,
                namespace=ns,
                respawn_delay=2.0,
                parameters=[configured_params,
                            {'use_sim_time': True}],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=[(pyexp(SingleSubstitution('/{@}/tf', '{@}', ns)), '/tf'), 
                            (pyexp(SingleSubstitution('/{@}/tf_static', '{@}', ns)), 'tf_static')]
            ),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                namespace=ns,
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=[(pyexp(SingleSubstitution('/{@}/tf', '{@}', ns)), '/tf'), 
                            (pyexp(SingleSubstitution('/{@}/tf_static', '{@}', ns)), 'tf_static'),
                            ('cmd_vel', 'cmd_vel_nav')],
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                namespace=ns,
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=[(pyexp(SingleSubstitution('/{@}/tf', '{@}', ns)), '/tf'), 
                            (pyexp(SingleSubstitution('/{@}/tf_static', '{@}', ns)), 'tf_static'),
                            ('/cmd_vel', pyexp(SingleSubstitution('/{@}/cmd_vel', '{@}', ns)))],
            ),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                namespace=ns,
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=[(pyexp(SingleSubstitution('/{@}/tf', '{@}', ns)), '/tf'), 
                            (pyexp(SingleSubstitution('/{@}/tf_static', '{@}', ns)), 'tf_static')],
            ),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                namespace=ns,
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=[(pyexp(SingleSubstitution('/{@}/tf', '{@}', ns)), '/tf'), 
                            (pyexp(SingleSubstitution('/{@}/tf_static', '{@}', ns)), 'tf_static')]
                + [('cmd_vel', 'cmd_vel_nav')],
            ),
            Node(
                package='nav2_collision_monitor',
                executable='collision_monitor',
                name='collision_monitor',
                output='screen',
                namespace=ns,
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=[(pyexp(SingleSubstitution('/{@}/tf', '{@}', ns)), '/tf'), 
                            (pyexp(SingleSubstitution('/{@}/tf_static', '{@}', ns)), 'tf_static')],
            ),
            Node(
                package='opennav_docking',
                executable='opennav_docking',
                name='docking_server',
                output='screen',
                namespace=ns,
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=[(pyexp(SingleSubstitution('/{@}/tf', '{@}', ns)), '/tf'), 
                            (pyexp(SingleSubstitution('/{@}/tf_static', '{@}', ns)), 'tf_static')],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                namespace=ns,
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'autostart': autostart}, {'node_names': lifecycle_nodes},
                            {'use_sim_time': True}],
            ),
        ],
    )

    ld = LaunchDescription([
        stdout_linebuf_envvar,
        declare_namespace_cmd,
        declare_use_sim_time_cmd,
        declare_params_file_cmd,
        declare_autostart_cmd,
        declare_use_composition_cmd,
        declare_container_name_cmd,
        declare_use_respawn_cmd,
        declare_log_level_cmd,
        load_nodes
    ])

    return ld