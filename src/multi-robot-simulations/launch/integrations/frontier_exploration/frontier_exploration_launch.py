# Jazzy-Multi-Robot-Sandbox for multi-robot research using ROS 2
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
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description(): 
    # get paths for gazebo simulator
    gazebo_package_dir = get_package_share_directory('ros_gz_sim')
    project_dir = get_package_share_directory('multi-robot-simulations')
    world_name_arg = DeclareLaunchArgument('world', default_value='worlds/high-end/urban_circuit_02.sdf')

    # Setup to launch the simulator and Gazebo world
    gazebo_launch_path = os.path.join(gazebo_package_dir, 'launch', 'gz_sim.launch.py')
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
        launch_arguments={
            'render_engine': 'ogre2',
            'use_sim_time': 'True',
            'gz_args': ['-r ', LaunchConfiguration('world')],
             'on_exit_shutdown': 'true' # -r run unpaused, -s runs without gui
            }.items()
    )

    # husky spawn path
    husky_launch_path = os.path.join(project_dir, 'launch', 'integrations', 'frontier_exploration', 'frontier_exploration_spawn_husky_launch.py')
    spawn_husky_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(husky_launch_path),
        launch_arguments={
            'namespace': 'robot_1',
            'x': '0.0',
            'y': '-18.0',
            'z': '1.05',
            'slam_config_file': 'robot_1_slam.yaml',
            'nav2_config_file': 'robot_1_nav2.yaml'
        }.items()
    )

    # Setup gz_bridge node
    ros_bridge_node = Node(
                        package='ros_gz_bridge',
                        namespace='ros_gz_bridge',
                        executable='parameter_bridge',
                        name='global_gz_bridge',
                        parameters=[{'use_sim_time': True}],
                        arguments=['/world/empty/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'
                                ],
                        remappings=[
                            ('/world/empty/clock', '/clock')
                        ],
    )

    joy_driver_node = Node(
            package='joy', 
            executable='joy_node', 
            name='joy_node',
            parameters=[{
                'device_id': 0,
                'deadzone': 0.3,
                'autorepeat_rate': 20.0
    }])
    
    joy_config = os.path.join(get_package_share_directory('teleop_twist_joy'), 'config', 'xbox.config.yaml')
    teleop_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[joy_config,
                    {
                        'use_sim_time': True
                    }],
        remappings={('/cmd_vel', '/robot_1/cmd_vel')},
    )

    rviz_node = Node(
                        package='rviz2',
                        executable='rviz2',
                        name='multi_robot_rviz',
                        namespace='',
                        parameters=[{'use_sim_time': True}],
                        arguments=["-d", os.path.join(project_dir,'config', 'integrations', 'frontier_discovery', 'all_robots.rviz')],
                        remappings={('/goal_pose', '/robot_1/goal_pose'),
                                    ('/initialpose', '/robot_1/initialpose')}
    )

    return LaunchDescription([
        world_name_arg,
        gz_sim_launch,
        ros_bridge_node,
        spawn_husky_1,
        rviz_node,
        joy_driver_node,
        teleop_joy_node
    ])