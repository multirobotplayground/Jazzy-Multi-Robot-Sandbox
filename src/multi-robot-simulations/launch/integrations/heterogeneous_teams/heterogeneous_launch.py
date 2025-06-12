# Jazzy-Multi-Robot-Sandbox for multi-robot research using ROS 2
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

    # create all arguments for this launch file
    world_name_arg = DeclareLaunchArgument('world', default_value='worlds/low-end/empty.sdf')

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
    husky_launch_path = os.path.join(project_dir, 'launch', 'integrations', 'heterogeneous_teams', 'heterogeneous_spawn_husky_ugv_launch.py')
    spawn_husky_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(husky_launch_path),
        launch_arguments={
            'namespace': 'robot_1',
            'x': '0.0',
            'y': '0.0',
            'z': '0.15'
        }.items()
    )

    spawn_husky_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(husky_launch_path),
        launch_arguments={
            'namespace': 'robot_2',
            'x': '-1.0',
            'y': '0.0',
            'z': '0.15'
        }.items()
    )

    x2_launch_path = os.path.join(project_dir, 'launch', 'integrations', 'heterogeneous_teams', 'heterogeneous_spawn_x2_ugv_launch.py')
    spawn_x2_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(x2_launch_path),
        launch_arguments={
            'namespace': 'robot_3',
            'x': '0.0',
            'y': '1.0',
            'z': '0.15'
        }.items()
    )

    spawn_x2_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(x2_launch_path),
        launch_arguments={
            'namespace': 'robot_4',
            'x': '-1.0',
            'y': '1.0',
            'z': '0.15'
        }.items()
    )

    x4_drone_launch_path = os.path.join(project_dir, 'launch', 'integrations', 'heterogeneous_teams', 'heterogeneous_spawn_x4_ufv_launch.py')
    spawn_x4_drone_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(x4_drone_launch_path),
        launch_arguments={
            'namespace': 'robot_5',
            'x': '0.0',
            'y': '2.0',
            'z': '0.15'
        }.items()
    )   

    spawn_x4_drone_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(x4_drone_launch_path),
        launch_arguments={
            'namespace': 'robot_6',
            'x': '-1.0',
            'y': '2.0',
            'z': '0.15'
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

    rviz_husky_1 = Node(
                        package='rviz2',
                        executable='rviz2',
                        name='multi_robot_rviz',
                        namespace='',
                        parameters=[{'use_sim_time': True}],
                        arguments=["-d", os.path.join(project_dir, 'config', 'integrations', 'heterogeneous_teams', 'husky_1.rviz')],
                        remappings={('/goal_pose', '/robot_1/goal_pose'),
                                    ('/initialpose', '/robot_1/initialpose')}
    )

    rviz_husky_2 = Node(
                        package='rviz2',
                        executable='rviz2',
                        name='multi_robot_rviz',
                        namespace='',
                        parameters=[{'use_sim_time': True}],
                        arguments=["-d", os.path.join(project_dir, 'config', 'integrations', 'heterogeneous_teams', 'husky_2.rviz')],
                        remappings={('/goal_pose', '/robot_2/goal_pose'),
                                    ('/initialpose', '/robot_2/initialpose')}
    )

    rviz_x2_1 = Node(
                        package='rviz2',
                        executable='rviz2',
                        name='multi_robot_rviz',
                        namespace='',
                        parameters=[{'use_sim_time': True}],
                        arguments=["-d", os.path.join(project_dir, 'config', 'integrations', 'heterogeneous_teams', 'x2_1.rviz')],
                        remappings={('/goal_pose', '/robot_3/goal_pose'),
                                    ('/initialpose', '/robot_3/initialpose')}
    )

    rviz_x2_2 = Node(
                        package='rviz2',
                        executable='rviz2',
                        name='multi_robot_rviz',
                        namespace='',
                        parameters=[{'use_sim_time': True}],
                        arguments=["-d", os.path.join(project_dir, 'config', 'integrations', 'heterogeneous_teams', 'x2_2.rviz')],
                        remappings={('/goal_pose', '/robot_4/goal_pose'),
                                    ('/initialpose', '/robot_4/initialpose')}
    )

    rviz_x4_1 = Node(
                        package='rviz2',
                        executable='rviz2',
                        name='multi_robot_rviz',
                        namespace='',
                        parameters=[{'use_sim_time': True}],
                        arguments=["-d", os.path.join(project_dir, 'config', 'integrations', 'heterogeneous_teams', 'x4_1.rviz')],
                        remappings={('/goal_pose', '/robot_5/goal_pose'),
                                    ('/initialpose', '/robot_5/initialpose')}
    )

    rviz_x4_2 = Node(
                        package='rviz2',
                        executable='rviz2',
                        name='multi_robot_rviz',
                        namespace='',
                        parameters=[{'use_sim_time': True}],
                        arguments=["-d", os.path.join(project_dir, 'config', 'integrations', 'heterogeneous_teams', 'x4_2.rviz')],
                        remappings={('/goal_pose', '/robot_6/goal_pose'),
                                    ('/initialpose', '/robot_6/initialpose')}
    )

    return LaunchDescription([
        world_name_arg,
        gz_sim_launch,
        ros_bridge_node,
        spawn_husky_1,
        spawn_husky_2,
        spawn_x2_1,
        spawn_x2_2,
        spawn_x4_drone_1,
        spawn_x4_drone_2,
        rviz_husky_1,
        rviz_husky_2,
        rviz_x2_1,
        rviz_x2_2,
        rviz_x4_1,
        rviz_x4_2,
        joy_driver_node,
        teleop_joy_node
    ])