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
import re
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression as pyexp

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
    cmd_vel_bridge = SingleSubstitution("/model/{@}/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist", 
                                        "{@}",
                                        ns)
    point_cloud_bridge = SingleSubstitution("/world/empty/model/{@}/link/base_link/sensor/front_laser/scan/points@"\
                                            "sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked", 
                                            "{@}",
                                            ns)
    lidar_scan_bridge = SingleSubstitution("/world/empty/model/{@}/link/base_link/sensor/front_laser/scan@"\
                                           "sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                                           "{@}",
                                            ns)
    imu_bridge = SingleSubstitution("/world/empty/model/{@}/link/base_link/sensor/imu_sensor/imu@"\
                                    "sensor_msgs/msg/Imu[gz.msgs.IMU",
                                    "{@}",
                                    ns)
    odometry_bridge = SingleSubstitution("/model/{@}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                                         "{@}",
                                         ns)
    global_localization_bridge = SingleSubstitution("/model/{@}/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose",
                                                    "{@}",
                                                    ns)
    joint_states = SingleSubstitution("/world/empty/model/{@}/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model",
                                                    "{@}",
                                                    ns)
    
    ros_bridge_node = Node(
                        package='ros_gz_bridge',
                        namespace='ros_gz_bridge',
                        executable='parameter_bridge',
                        name=pyexp(SingleSubstitution('{@}_gz_bridge', '{@}', ns)),
                        parameters=[{'use_sim_time': use_sim_time}],
                        arguments=[pyexp(cmd_vel_bridge),
                                   pyexp(point_cloud_bridge),
                                   pyexp(lidar_scan_bridge),
                                   pyexp(imu_bridge),
                                   pyexp(odometry_bridge),
                                   pyexp(global_localization_bridge),
                                   pyexp(joint_states)
                                ],
                        remappings=[
                            (pyexp(SingleSubstitution('/model/{@}/cmd_vel', '{@}', ns)), pyexp(SingleSubstitution('/{@}/cmd_vel', '{@}', ns))),
                            (pyexp(SingleSubstitution('/world/empty/model/{@}/link/base_link/sensor/front_laser/scan/points', '{@}', ns)), pyexp(SingleSubstitution('/{@}/lidar/points', "{@}", ns))),
                            (pyexp(SingleSubstitution('/world/empty/model/{@}/link/base_link/sensor/front_laser/scan', "{@}", ns)), pyexp(SingleSubstitution('/{@}/lidar/scan', "{@}", ns))),
                            (pyexp(SingleSubstitution('/world/empty/model/{@}/link/base_link/sensor/imu_sensor/imu', "{@}", ns)), pyexp(SingleSubstitution('/{@}/imu', "{@}", ns))),
                            (pyexp(SingleSubstitution('/model/{@}/odometry', "{@}", ns)), pyexp(SingleSubstitution('/{@}/odometry', "{@}", ns))),
                            (pyexp(SingleSubstitution('/model/{@}/pose', "{@}", ns)), pyexp(SingleSubstitution('/{@}/pose', "{@}", ns))),
                            (pyexp(SingleSubstitution('/world/empty/model/{@}/joint_state', "{@}", ns)), pyexp(SingleSubstitution('/{@}/joint_states', "{@}", ns)))
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
                             'frame_prefix': pyexp(SingleSubstitution('{@}/', "{@}", ns)),
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
                    'topic': pyexp(SingleSubstitution('/{@}/robot_description', "{@}", ns)),
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
                remappings=[("/pose", pyexp(SingleSubstitution('/{@}/pose', '{@}', ns)))]
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