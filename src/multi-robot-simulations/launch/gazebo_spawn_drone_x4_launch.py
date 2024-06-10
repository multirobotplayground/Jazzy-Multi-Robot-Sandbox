#import sys
import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

from launch_ros.actions import Node as create_node_description
#from launch.launch_description_sources import PythonLaunchDescriptionSource as load_python_launch_file
# from launch.actions import IncludeLaunchDescription as include_another_launch_file 
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
#from launch.substitutions import PathJoinSubstitution
#from launch.substitutions import TextSubstitution
from launch.substitutions import PythonExpression as pyexp

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
    global_localization_bridge = SingleSubstitution("/model/{@}/pose@geometry_msgs/msg/Pose[gz.msgs.Pose",
                                                    "{@}",
                                                    ns)
        
    # Setup gz_bridge node
    ros_bridge_node = create_node_description(
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
                                   pyexp(global_localization_bridge)
                                ],
                        remappings=[
                            (pyexp(SingleSubstitution('/model/{@}/cmd_vel', '{@}', ns)), pyexp(SingleSubstitution('/{@}/cmd_vel', '{@}', ns))),
                            (pyexp(SingleSubstitution('/world/empty/model/{@}/link/base_link/sensor/camera_front/depth_image', '{@}', ns)), pyexp(SingleSubstitution('/{@}/camera/depth_image', "{@}", ns))),
                            (pyexp(SingleSubstitution('/world/empty/model/{@}/link/base_link/sensor/camera_front/image', '{@}', ns)), pyexp(SingleSubstitution('/{@}/camera/image', "{@}", ns))),
                            (pyexp(SingleSubstitution('/world/empty/model/{@}/link/base_link/sensor/camera_front/camera_info', '{@}', ns)), pyexp(SingleSubstitution('/{@}/camera/camera_info', "{@}", ns))),
                            (pyexp(SingleSubstitution('/world/empty/model/{@}/link/base_link/sensor/camera_front/points', '{@}', ns)), pyexp(SingleSubstitution('/{@}/camera/points', "{@}", ns))),
                            (pyexp(SingleSubstitution('/world/empty/model/{@}/link/base_link/sensor/imu_sensor/imu', "{@}", ns)), pyexp(SingleSubstitution('/{@}/imu', "{@}", ns))),
                            (pyexp(SingleSubstitution('/model/{@}/odometry', "{@}", ns)), pyexp(SingleSubstitution('/{@}/odometry', "{@}", ns))),
                            (pyexp(SingleSubstitution('/model/{@}/pose', "{@}", ns)), pyexp(SingleSubstitution('/{@}/pose', "{@}", ns)))
                        ],
    )

    # read robot description file
    sdf_file = os.path.join(os.getenv('GZ_SIM_RESOURCE_PATH'), 'robots', 'X4_GPS_RGBD', 'model.sdf')
    with open(sdf_file, 'r') as input_file:
        robot_description = input_file.read()
    robot_description = robot_description.replace('\n', '').replace('\t', '').replace('\r', '').replace('\'','')

    # create a parameter to hold the robot description from the file
    robot_state_publisher = create_node_description(
                package='robot_state_publisher',
                namespace=ns,
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'use_sim_time': True, 
                             'robot_description': pyexp(SingleSubstitution(robot_description, 'REPLACE_THIS_NAMESPACE', ns)),
                             'frame_prefix': pyexp(SingleSubstitution('{@}/', "{@}", ns))}],
                arguments=[])
    
    robot_joint_state_publisher = create_node_description(
                package='joint_state_publisher',
                namespace=ns,
                executable='joint_state_publisher',
                name='jont_state_publisher',
                output='screen',
                remappings=[
                    ('/robot_description', pyexp(SingleSubstitution('/{@}/robot_description', "{@}", ns)))
                ],
                parameters=[{'frame_prefix': pyexp(SingleSubstitution('{@}/', "{@}", ns))}],
                arguments=[])

    spawn = create_node_description(
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

    return LaunchDescription([
        x,
        y,
        z,
        robot_namespace,
        robot_state_publisher,
        robot_joint_state_publisher,
        ros_bridge_node,
        spawn
    ])