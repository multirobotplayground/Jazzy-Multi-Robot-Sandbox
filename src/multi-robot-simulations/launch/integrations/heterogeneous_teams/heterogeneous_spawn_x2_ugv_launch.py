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
import re
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression as pyexp

def generate_launch_description(): 
    robot_namespace = DeclareLaunchArgument('namespace', default_value='robot_0')
    x = DeclareLaunchArgument('x', default_value='0.0')
    y = DeclareLaunchArgument('y', default_value='0.0')
    z = DeclareLaunchArgument('z', default_value='0.2')
    
    ns = LaunchConfiguration('namespace')
    x_val = LaunchConfiguration('x')
    y_val = LaunchConfiguration('y')
    z_val = LaunchConfiguration('z')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    default_tf_hz = LaunchConfiguration('default_tf_hz', default=50.0)

    # Must find a better way to do this substituition
    cmd_vel_bridge = ["/model/", ns, "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist"]
    point_cloud_bridge = ["/world/empty/model/", ns, "/link/base_link/sensor/front_laser/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked"]
    lidar_scan_bridge = ["/world/empty/model/", ns, "/link/base_link/sensor/front_laser/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"]
    imu_bridge = ["/world/empty/model/", ns, "/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU"]
    odometry_bridge = ["/model/", ns, "/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry"]
    global_localization_bridge = ["/model/", ns, "/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose"]
    joint_states = ["/world/empty/model/", ns, "/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model"]

    ros_bridge_node = Node(
                        package='ros_gz_bridge',
                        namespace='ros_gz_bridge',
                        executable='parameter_bridge',
                        name=[ns, '_gz_bridge'],
                        parameters=[{'use_sim_time': use_sim_time}],
                        arguments=[cmd_vel_bridge,
                                   point_cloud_bridge,
                                   lidar_scan_bridge,
                                   imu_bridge,
                                   odometry_bridge,
                                   global_localization_bridge,
                                   joint_states
                                ],
                        remappings=[
                            (['/model/', ns, '/cmd_vel'], ['/', ns, '/cmd_vel']),
                            (['/world/empty/model/', ns, '/link/base_link/sensor/front_laser/scan/points'], ['/', ns, '/lidar/points']),
                            (['/world/empty/model/', ns, '/link/base_link/sensor/front_laser/scan'], ['/', ns, '/lidar/scan']),
                            (['/world/empty/model/', ns, '/link/base_link/sensor/imu_sensor/imu'], ['/', ns, '/imu']),
                            (['/model/', ns, '/odometry'], ['/', ns, '/odometry']),
                            (['/model/', ns, '/pose'], ['/', ns, '/pose']),
                            (['/world/empty/model/', ns, '/joint_state'], ['/', ns, '/joint_states'])
                        ],
    )

    sdf_file = os.path.join(os.getenv('GZ_SIM_RESOURCE_PATH'), 'robots', 'X2_Config_6', 'model.sdf')
    with open(sdf_file, 'r') as input_file:
        robot_description = input_file.read()

    robot_description = robot_description.replace('REPLACE_RESOURCE_PATH', os.getenv('GZ_SIM_RESOURCE_PATH'))
    robot_state_publisher = Node(
                package='robot_state_publisher',
                namespace=ns,
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'robot_description': robot_description,
                             'frame_prefix': [ns, '/'],
                             'use_sim_time': use_sim_time,
                             'publish_frequency': default_tf_hz}],
                arguments=[])

    spawn = Node(
                package='ros_gz_sim',
                namespace=ns,
                name='spawn_node',
                executable='create',
                parameters=[{
                    'name': ns,
                    'x': x_val,
                    'z': z_val,
                    'y': y_val,
                    'topic': ['/', ns, '/robot_description'],
                    'use_sim_time': use_sim_time}],
                 output='screen')

    pose_tf_publisher = Node(
                package='multi-robot-simulations',
                namespace=ns,
                executable='multi_robot_simulation_main',
                output='screen',
                name='odom_publisher',
                parameters=[{'use_sim_time': use_sim_time,
                             "hz": 50}],
                remappings=[("/pose", ['/', ns, '/pose'])]
    )

    return LaunchDescription([
        x,
        y,
        z,
        robot_namespace,
        pose_tf_publisher,
        spawn,
        ros_bridge_node,
        robot_state_publisher
    ])