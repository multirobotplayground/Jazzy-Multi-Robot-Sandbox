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
from launch.substitutions import AndSubstitution, NotSubstitution, LaunchConfiguration, PathJoinSubstitution
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml

def generate_launch_description(): 
    arg_robot_namespace = DeclareLaunchArgument('namespace', default_value='robot_0')
    arg_x = DeclareLaunchArgument('x', default_value='0.0')
    arg_y = DeclareLaunchArgument('y', default_value='0.0')
    arg_z = DeclareLaunchArgument('z', default_value='0.2')
    arg_slam_config_file = DeclareLaunchArgument('slam_config_file', default_value='robot_1_slam.yaml')
    arg_nav2_config_file = DeclareLaunchArgument('nav2_config_file', default_value='robot_1_nav2.yaml')
    arg_declare_autostart_cmd = DeclareLaunchArgument('autostart', default_value='true')
    arg_declare_use_lifecycle_manager = DeclareLaunchArgument('use_lifecycle_manager', default_value='false')

    ns = LaunchConfiguration('namespace')
    x_val = LaunchConfiguration('x')
    y_val = LaunchConfiguration('y')
    z_val = LaunchConfiguration('z')
    autostart = LaunchConfiguration('autostart')
    use_lifecycle_manager = LaunchConfiguration('use_lifecycle_manager')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    default_tf_hz = LaunchConfiguration('default_tf_hz', default=50.0)
    slam_config_file = LaunchConfiguration('slam_config_file', default='robot_1_slam.yaml')
    nav2_config_file = LaunchConfiguration('nav2_config_file', default='robot_1_nav2.yaml')

    ros_bridge_node = Node(
                        package='ros_gz_bridge',
                        namespace='ros_gz_bridge',
                        executable='parameter_bridge',
                        name=[ns, '_gz_bridge'],
                        parameters=[{'use_sim_time': use_sim_time}],
                        arguments=[['/model/', ns, '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
                                   ['/world/empty/model/', ns, '/link/sensor_rack/sensor/front_lidar/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'],
                                   ['/world/empty/model/', ns, '/link/sensor_rack/sensor/front_lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
                                   ['/world/empty/model/', ns, '/link/sensor_rack/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'],
                                   ['/model/', ns, '/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
                                   ['/model/', ns, '/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose'],
                                   ['/world/empty/model/', ns, '/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model']
                                ],
                        remappings=[
                            (['/model/', ns, '/cmd_vel'],['/', ns , '/cmd_vel']),
                            (['/world/empty/model/', ns, '/link/sensor_rack/sensor/front_lidar/scan/points'], ['/', ns, '/lidar/points']),
                            (['/world/empty/model/', ns, '/link/sensor_rack/sensor/front_lidar/scan'],['/', ns, '/lidar/scan']),
                            (['/world/empty/model/', ns, '/link/sensor_rack/sensor/imu_sensor/imu'],['/', ns, '/imu']),
                            (['/model/', ns, '/odometry'],['/', ns, '/odometry']),
                            (['/model/', ns, '/pose'],['/', ns, '/pose']),
                            (['/world/empty/model/', ns, '/joint_state'],['/', ns, '/joint_states']),
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
                output='log',
                parameters=[{'robot_description': robot_description,
                             'frame_prefix': [ns, '/'],
                             'use_sim_time': use_sim_time,
                             'publish_frequency': default_tf_hz}],
                arguments=[])

    common_frame_publisher = Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='common_frame',
                output='log',
                namespace=ns,
                remappings=[],
                parameters=[{'use_sim_time': use_sim_time}],
                arguments=['0', '0', '0', '0', '0', '0', 'global', [ns, '/map']])

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
                 output='log')

    param_substitutions = {'autostart': autostart}
    configured_params = ParameterFile(
        RewrittenYaml(
            # <<<<<<<<<<<<<<<<<< HERE IS THE CASE WHERE THE NATIVE SUBSTITUTION FAILS >>>>>>>>>>>>>>>>>>>>>>>>
            # MUST FORCE PYTHON EXPRESSION FOR THE FILE PATH
            source_file=PathJoinSubstitution([get_package_share_directory('multi-robot-simulations'), 
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
        remappings=[('/map', ['/', ns, '/map']),
                    ('/pose', ['/', ns, '/pose_slam'])],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='log',
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
            start_state='configuring',
            goal_state='inactive',
            entities=[
                LogInfo(msg='[LifecycleLaunch] Slamtoolbox node is activating.'),
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
                output='log',
                name='odom_publisher',
                parameters=[{'use_sim_time': use_sim_time,
                             'hz': 50}],
                remappings=[('/pose', ['/', ns, '/pose'])]
    )

    nav_launch_path = os.path.join(get_package_share_directory('multi-robot-simulations'), 'launch', 
                                   'integrations', 'intermittent_comm', 'intermittent_comm_nav2_launch.py')
    nav2_launch = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(nav_launch_path), 
                        launch_arguments={
                            'namespace': ns,
                            # THIS MUST BE A STRING
                            'use_sim_time': 'True',
                            'params_file': PathJoinSubstitution([get_package_share_directory('multi-robot-simulations'), 
                                                                 'config', 'integrations', 'slam_toolbox_and_nav2', nav2_config_file]),
                        }.items()
    )

    ground_segmentation_path = os.path.join(get_package_share_directory('point_cloud_segmentation'), 
                                            'launch', 'ground_segmentation_launch.py')
    ground_segmentation_launch = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(ground_segmentation_path), 
                        launch_arguments={
                            'namespace': ns,
                            # THIS MUST BE A STRING
                            'use_sim_time': 'True'
                        }.items()
    )

    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan_node',
        namespace=ns,
        output='log',
        parameters=[{'use_sim_time': use_sim_time,
                     'max_height': 2.0,
                     'min_height': -2.0}],
        remappings=[(['/', ns, '/cloud_in'], ['/',ns,'/segmented_cloud_pure']),
                    (['/', ns, '/scan'], ['/', ns, '/lidar/projected_cloud_scan'])]  
    )

    frontier_discovery_node = Node(
        package='frontier_exploration',
        executable='frontier_discovery_node',
        namespace=ns,
        name='frontier_discovery_node',
        output='log',
        parameters=[{
            'id': 0,  # Optionally set per-robot
            'max_lidar_range': 100.0,
            'rate': 2.0,
            'queue_size': 2,
            'use_sim_time': use_sim_time
        }],
        remappings=[(['/', ns, '/c_space'], ['/',ns,'/filtered_for_frontier_exploration'])]
    )

    occupancy_grid_filter_node = Node(
        package='frontier_exploration',
        executable='occupancy_grid_filter_node',
        namespace=ns,
        name='occupancy_grid_filter_node',
        output='log',
        parameters=[{'use_sim_time': use_sim_time,
                     'obstacle_inflation_radius_meters': 0.0}],
        remappings=[(['/', ns, '/input_occupancy_grid'], ['/', ns, '/map'])]
    )

    occupancy_grid_filter_frontiers_node = Node(
        package='frontier_exploration',
        executable='occupancy_grid_filter_node',
        namespace=ns,
        name='occupancy_grid_filter_node',
        output='log',
        parameters=[{'use_sim_time': use_sim_time,
                     'obstacle_inflation_radius_meters': 0.9}],
        remappings=[(['/', ns, '/input_occupancy_grid'], ['/', ns, '/map']),
                    (['/', ns, '/filtered_occupancy_grid'], ['/', ns, '/filtered_for_frontier_exploration'])]
    )

    frontier_exploration_node = Node(
        package='frontier_exploration',
        executable='frontier_exploration_node',
        namespace=ns,
        name='frontier_exploration_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[]
    )

    mock_communication_node = Node(
        package='mock_communication',
        executable='mock_com_node',
        namespace=ns,
        name='mock_comm_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[]
    )

    return LaunchDescription([
        arg_x,
        arg_y,
        arg_z,
        arg_slam_config_file,
        arg_nav2_config_file,
        arg_robot_namespace,
        arg_declare_autostart_cmd,
        arg_declare_use_lifecycle_manager,
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
        ground_segmentation_launch,
        frontier_discovery_node,
        occupancy_grid_filter_node,
        occupancy_grid_filter_frontiers_node,
        frontier_exploration_node,
        mock_communication_node
        ])