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

    # GZ TOPIC BRIDGE WITH SUBSTITUTION
    cmd_vel_bridge = SingleSubstitution("/model/{@}/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist", 
                                        "{@}", 
                                        ns)
    point_cloud_bridge = SingleSubstitution("/world/empty/model/{@}/link/base_link/sensor/camera_front/points@"\
                                            "sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked", 
                                            "{@}", 
                                            ns)
    camera_depth_image_bridge = SingleSubstitution("/world/empty/model/{@}/link/base_link/sensor/camera_front/depth_image@"\
                                             "sensor_msgs/msg/Image[gz.msgs.Image",
                                             "{@}",
                                             ns)
    camera_image_bridge = SingleSubstitution("/world/empty/model/{@}/link/base_link/sensor/camera_front/image@"\
                                             "sensor_msgs/msg/Image[gz.msgs.Image",
                                             "{@}",
                                             ns)
    camera_info_bridge = SingleSubstitution("/world/empty/model/{@}/link/base_link/sensor/camera_front/camera_info@"
                                            "sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
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
    
    # Setup gz_bridge node
    ros_bridge_node = Node(
                        package='ros_gz_bridge',
                        namespace='ros_gz_bridge',
                        executable='parameter_bridge',
                        name=pyexp(SingleSubstitution('{@}_gz_bridge', '{@}', ns)),
                        parameters=[],
                        arguments=[pyexp(cmd_vel_bridge),
                                   pyexp(point_cloud_bridge),
                                   pyexp(camera_depth_image_bridge),
                                   pyexp(camera_image_bridge),
                                   pyexp(camera_info_bridge),
                                   pyexp(imu_bridge),
                                   pyexp(odometry_bridge),
                                   pyexp(global_localization_bridge),
                                   pyexp(joint_states)
                                ],
                        remappings=[
                            (pyexp(SingleSubstitution('/model/{@}/cmd_vel', '{@}', ns)), pyexp(SingleSubstitution('/{@}/cmd_vel', '{@}', ns))),
                            (pyexp(SingleSubstitution('/world/empty/model/{@}/link/base_link/sensor/camera_front/depth_image', '{@}', ns)), pyexp(SingleSubstitution('/{@}/camera/depth_image', "{@}", ns))),
                            (pyexp(SingleSubstitution('/world/empty/model/{@}/link/base_link/sensor/camera_front/image', '{@}', ns)), pyexp(SingleSubstitution('/{@}/camera/image', "{@}", ns))),
                            (pyexp(SingleSubstitution('/world/empty/model/{@}/link/base_link/sensor/camera_front/camera_info', '{@}', ns)), pyexp(SingleSubstitution('/{@}/camera/camera_info', "{@}", ns))),
                            (pyexp(SingleSubstitution('/world/empty/model/{@}/link/base_link/sensor/camera_front/points', '{@}', ns)), pyexp(SingleSubstitution('/{@}/camera/points', "{@}", ns))),
                            (pyexp(SingleSubstitution('/world/empty/model/{@}/link/base_link/sensor/imu_sensor/imu', "{@}", ns)), pyexp(SingleSubstitution('/{@}/imu', "{@}", ns))),
                            (pyexp(SingleSubstitution('/model/{@}/odometry', "{@}", ns)), pyexp(SingleSubstitution('/{@}/odometry', "{@}", ns))),
                            (pyexp(SingleSubstitution('/model/{@}/pose', "{@}", ns)), pyexp(SingleSubstitution('/{@}/pose', "{@}", ns))),
                            (pyexp(SingleSubstitution('/world/empty/model/{@}/joint_state', "{@}", ns)), pyexp(SingleSubstitution('/{@}/joint_states', "{@}", ns)))
                        ],
    )

    # read robot description file
    sdf_file = os.path.join(os.getenv('GZ_SIM_RESOURCE_PATH'), 'robots', 'X4_GPS_RGBD', 'model.sdf')
    with open(sdf_file, 'r') as input_file:
        robot_description = input_file.read()

    # clear description from unwanted characters
    robot_description = robot_description.replace('\n', '').replace('\t', '').replace('\r', '').replace('\'','')

    # replace the resource path since I've installed the models manually
    robot_description = robot_description.replace('REPLACE_RESOURCE_PATH', os.getenv('GZ_SIM_RESOURCE_PATH'))

    # create a parameter to hold the robot description from the file
    robot_state_publisher = Node(
                package='robot_state_publisher',
                namespace=ns,
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'robot_description': pyexp(SingleSubstitution(robot_description, 'REPLACE_THIS_NAMESPACE', ns)),
                             'frame_prefix': pyexp(SingleSubstitution('{@}/', "{@}", ns)),
                             'use_sim_time': use_sim_time,
                             'publish_frequency': default_tf_hz}],
                arguments=[])

    spawn = Node(
                package='ros_gz_sim', 
                namespace='ros_gz_sim',
                executable='create',
                parameters=[{
                    'name': ns,
                    'x': x_val,
                    'z': z_val,
                    'y': y_val,
                    'topic': pyexp(SingleSubstitution('/{@}/robot_description', "{@}", ns))}],
                 output='screen')
    
    pose_tf_publisher = Node(
                package='multi-robot-simulations',
                namespace=ns,
                executable='multi_robot_simulation_main',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time,
                "hz": 50}],
                remappings=[("/pose", pyexp(SingleSubstitution('/{@}/pose', '{@}', ns)))]
    )

    return LaunchDescription([
        x,
        y,
        z,
        robot_namespace,
        robot_state_publisher,
        ros_bridge_node,
        pose_tf_publisher,
        spawn
    ])