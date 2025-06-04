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

    # create all arguments for this launch file
    launch_args = DeclareLaunchArgument('expand_gz_topic_names', default_value='True')
    world_name_arg = DeclareLaunchArgument('world', default_value='worlds/low-end/empty.sdf')
    bridge_config_arg = DeclareLaunchArgument('config_file', default_value='gz_bridge.yaml')

    # Setup to launch the simulator and Gazebo world
    gazebo_launch_path = os.path.join(gazebo_package_dir, 'launch', 'gz_sim.launch.py')
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
        launch_arguments={
            'gz_args': ['-r ', LaunchConfiguration('world')],
            'render_engine': 'ogre2'
            }.items()
    )

    # Setup gz_bridge node
    ros_bridge_node = Node(
                        package='ros_gz_bridge',
                        namespace='ros_gz_bridge',
                        executable='parameter_bridge',
                        name='sim',
                        parameters=[],
                        arguments=['/model/robot_0/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                                   '/world/empty/model/robot_0/link/sensor_rack/sensor/front_lidar/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                                   '/world/empty/model/robot_0/link/sensor_rack/sensor/front_lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                                   '/world/empty/model/robot_0/link/sensor_rack/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                                   '/model/robot_0/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
                        remappings=[
                            ('/model/robot_0/cmd_vel', '/robot_0/cmd_vel'),
                            ('/world/empty/model/robot_0/link/sensor_rack/sensor/front_lidar/scan/points', '/robot_0/lidar/points'),
                            ('/world/empty/model/robot_0/link/sensor_rack/sensor/front_lidar/scan', '/robot_0/lidar/scan'),
                            ('/world/empty/model/robot_0/link/sensor_rack/sensor/imu_sensor/imu', '/robot_0/imu'),
                            ('/model/robot_0/odometry', '/robot_0/odometry')
                        ],
    )

    # read robot description file
    husky_sdf_file = os.path.join(os.getenv('GZ_SIM_RESOURCE_PATH'), 'robots', 'CTU_CRAS_NORLAB_HUSKY_SENSOR_CONFIG_1', 'model.sdf')
    with open(husky_sdf_file, 'r') as input_file:
        robot_description = input_file.read()
    robot_description = robot_description.replace('REPLACE_RESOURCE_PATH', os.getenv('GZ_SIM_RESOURCE_PATH'))

    # create a parameter to hold the robot description from the file
    robot_state_publisher = Node(
                package='robot_state_publisher',
                namespace='robot_0',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'use_sim_time': True, 
                             'robot_description': robot_description,
                             'frame_prefix': 'robot_0/'}],
                arguments=[])
    
    robot_joint_state_publisher = Node(
                package='joint_state_publisher',
                namespace='robot_0',
                executable='joint_state_publisher',
                name='jont_state_publisher',
                output='screen',
                remappings=[
                    ('/robot_description', '/robot_0/robot_description')
                ],
                parameters=[{'frame_prefix': 'robot_0/'}],
                arguments=[])

    spawn = Node(
                package='ros_gz_sim', 
                namespace='ros_gz_sim',
                executable='create',
                parameters=[{
                    'name': 'robot_0',
                    'x': 0.0,
                    'z': 0.5,
                    'Y': 0.0,
                    'topic': '/robot_0/robot_description'}],
                 output='screen')

    return LaunchDescription([
        launch_args,
        world_name_arg,
        bridge_config_arg,
        gz_sim_launch,
        ros_bridge_node,
        robot_state_publisher,
        robot_joint_state_publisher,
        spawn
    ])