import sys
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

from launch_ros.actions import Node as create_node_description
from launch.launch_description_sources import PythonLaunchDescriptionSource as load_python_launch_file
from launch.actions import IncludeLaunchDescription as include_another_launch_file 
from launch.substitutions import LaunchConfiguration as get_set_argument_val
from launch.substitutions import PathJoinSubstitution as concatenate_strings
from launch.actions import DeclareLaunchArgument as create_input_argument

def generate_launch_description(): 
    ####################################################################################
    # get worlds path
    project_package_dir = get_package_share_directory('gazebo-robots-descriptions')

    # get paths for gazebo simulator
    gazebo_package_dir = get_package_share_directory('ros_gz_sim')
    ####################################################################################


    ####################################################################################
    # create all arguments for this launch file
    launch_args = create_input_argument('expand_gz_topic_names', default_value='True')
    world_name_arg = create_input_argument('world', default_value='worlds/low-end/empty.sdf')
    bridge_config_arg = create_input_argument('config_file', default_value='gz_bridge.yaml')

    # setup all configurations in it so you can grab the values when launching the nodes
    get_set_argument_val('expand_gz_topic_names')
    get_set_argument_val('world')
    get_set_argument_val('config_file')
    ####################################################################################


    # Setup to launch the simulator and Gazebo world
    gazebo_launch_path = os.path.join(gazebo_package_dir, 'launch', 'gz_sim.launch.py')
    gz_sim_launch = include_another_launch_file(
        load_python_launch_file(gazebo_launch_path),
        launch_arguments={
            'gz_args': get_set_argument_val('world'),
            'render_engine': 'ogre2'
            }.items()
    )

    # Setup gz_bridge node
    ros_bridge_node = create_node_description(
                        package='ros_gz_bridge',
                        namespace='ros_gz_bridge',
                        executable='parameter_bridge',
                        name='sim',
                        remappings=[
                            # ('/input/pose', '/turtlesim1/turtle1/pose')
                        ],
                        parameters=[
                            {'config_file': concatenate_strings([project_package_dir, 'config', get_set_argument_val('config_file')])}
                        ]  
                    )
    
    # read robot description file
    husky_sdf_file = os.path.join(os.getenv('GZ_SIM_RESOURCE_PATH'), 'robots', 'CTU_CRAS_NORLAB_HUSKY_SENSOR_CONFIG_1', 'model.sdf')
    with open(husky_sdf_file, 'r') as input_file:
        robot_description = input_file.read()

    # create a parameter to hold the robot description from the file
    robot_state_publisher = create_node_description(
                package='robot_state_publisher',
                namespace='robot_0',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'use_sim_time': True, 
                             'robot_description': robot_description,
                             'frame_prefix': 'robot_0/'}],
                arguments=[])
    
    robot_joint_state_publisher = create_node_description(
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

    spawn = create_node_description(
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

    teleop_launch_path = os.path.join(project_package_dir, 'launch', 'teleop-robot-id.py')
    teleop_launch = include_another_launch_file(
        load_python_launch_file(teleop_launch_path),
        launch_arguments={
            'namespace': 'robot_0'
        }.items()
    )


    return LaunchDescription([
        launch_args,
        world_name_arg,
        bridge_config_arg,
        gz_sim_launch,
        ros_bridge_node,
        robot_state_publisher,
        robot_joint_state_publisher,
        spawn,
        teleop_launch
    ])