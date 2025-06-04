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
from launch_ros.actions import Node, LifecycleNode
from launch.conditions import IfCondition
from launch.events import matches_action
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler
from launch.substitutions import AndSubstitution, NotSubstitution, LaunchConfiguration, PathJoinSubstitution, PythonExpression as pyexp
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

    # Must find a better way to do this substituition, since this is rather naive
    cmd_vel_bridge = SingleSubstitution("/model/{@}/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist", 
                                        "{@}", 
                                        ns)
    point_cloud_bridge = SingleSubstitution("/world/empty/model/{@}/link/sensor_rack/sensor/front_lidar/scan/points@"\
                                            "sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked", 
                                            "{@}", 
                                            ns)
    lidar_scan_bridge = SingleSubstitution("/world/empty/model/{@}/link/sensor_rack/sensor/front_lidar/scan@"\
                                           "sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                                           "{@}", 
                                            ns)
    imu_bridge = SingleSubstitution("/world/empty/model/{@}/link/sensor_rack/sensor/imu_sensor/imu@"\
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
                            (pyexp(SingleSubstitution('/world/empty/model/{@}/link/sensor_rack/sensor/front_lidar/scan/points', '{@}', ns)), pyexp(SingleSubstitution('/{@}/lidar/points', "{@}", ns))),
                            (pyexp(SingleSubstitution('/world/empty/model/{@}/link/sensor_rack/sensor/front_lidar/scan', "{@}", ns)), pyexp(SingleSubstitution('/{@}/lidar/scan', "{@}", ns))),
                            (pyexp(SingleSubstitution('/world/empty/model/{@}/link/sensor_rack/sensor/imu_sensor/imu', "{@}", ns)), pyexp(SingleSubstitution('/{@}/imu', "{@}", ns))),
                            (pyexp(SingleSubstitution('/model/{@}/odometry', "{@}", ns)), pyexp(SingleSubstitution('/{@}/odometry', "{@}", ns))),
                            (pyexp(SingleSubstitution('/model/{@}/pose', "{@}", ns)), pyexp(SingleSubstitution('/{@}/pose', "{@}", ns))),
                            (pyexp(SingleSubstitution('/world/empty/model/{@}/joint_state', "{@}", ns)), pyexp(SingleSubstitution('/{@}/joint_states', "{@}", ns)))
                        ],
    )

    sdf_file = os.path.join(os.getenv('GZ_SIM_RESOURCE_PATH'), 'robots', 'CTU_CRAS_NORLAB_HUSKY_SENSOR_CONFIG_1', 'model.sdf')
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

    common_frame_publisher = Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name="common_frame",
                output='screen',
                namespace=ns,
                remappings=[],
                parameters=[{'use_sim_time': use_sim_time}],
                arguments=["0", "0", "0", "0", "0", "0", 'global', pyexp(SingleSubstitution('{@}/map', "{@}", ns))])

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

    autostart = LaunchConfiguration('autostart')
    use_lifecycle_manager = LaunchConfiguration("use_lifecycle_manager")

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the slamtoolbox. '
                    'Ignored when use_lifecycle_manager is true.')
    declare_use_lifecycle_manager = DeclareLaunchArgument(
        'use_lifecycle_manager', default_value='false',
        description='Enable bond connection during node activation')

    param_substitutions = {'autostart': autostart}
    slam_config_file = pyexp(SingleSubstitution('{@}_slam.yaml', '{@}', ns))
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=PathJoinSubstitution([get_package_share_directory("multi-robot-simulations"), 
                                              'config', 'integrations', 'slam_toolbox_and_nav2', slam_config_file]),
            root_key=ns,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    start_async_slam_toolbox_node = LifecycleNode(
        parameters=[
          configured_params,
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

    pose_tf_publisher = Node(
                package='multi-robot-simulations',
                namespace=ns,
                executable='multi_robot_simulation_main',
                output='screen',
                name='odom_publisher',
                parameters=[{'use_sim_time': use_sim_time,
                             "hz": 50}],
                remappings=[("/pose", pyexp(SingleSubstitution('/{@}/pose', '{@}', ns))),]
    )

    param_file = pyexp(SingleSubstitution('{@}_nav2.yaml', '{@}', ns))
    nav_launch_path = os.path.join(get_package_share_directory('multi-robot-simulations'), 'launch', 
                                   'integrations', 'slam_toolbox_and_nav2', 'nav2_launch.py')
    nav2_launch = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(nav_launch_path), 
                        launch_arguments={
                            'namespace': ns,
                            'use_sim_time': 'True',
                            'params_file': PathJoinSubstitution([get_package_share_directory('multi-robot-simulations'), 
                                                                 'config', 'integrations', 'slam_toolbox_and_nav2', param_file]),
                        }.items()
    )

    ground_segmentation_path = os.path.join(get_package_share_directory('point_cloud_segmentation'), 
                                            'launch', 'ground_segmentation_launch.py')
    ground_segmentation_launch = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(ground_segmentation_path), 
                        launch_arguments={
                            'namespace': ns,
                            'use_sim_time': 'True'
                        }.items()
    )

    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan_node',
        namespace=ns,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'max_height': 2.0}],
        remappings=[(pyexp(SingleSubstitution('/{@}/cloud_in', '{@}', ns)), pyexp(SingleSubstitution('/{@}/segmented_cloud_pure', '{@}', ns))),
                    (pyexp(SingleSubstitution('/{@}/scan', '{@}', ns)), pyexp(SingleSubstitution('/{@}/lidar/projected_cloud_scan', '{@}', ns)))]        
    )

    return LaunchDescription([
        x,
        y,
        z,
        robot_namespace,
        declare_autostart_cmd,
        declare_use_lifecycle_manager,
        common_frame_publisher,
        pose_tf_publisher,
        spawn,
        ros_bridge_node,
        robot_state_publisher,
        start_async_slam_toolbox_node,
        configure_slam_node_event,
        activate_slam_node_event,
        nav2_launch,
        pointcloud_to_laserscan,
        ground_segmentation_launch
    ])