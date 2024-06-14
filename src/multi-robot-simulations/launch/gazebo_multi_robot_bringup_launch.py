#import sys
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

#from launch_ros.actions import Node as create_node_description
from launch.launch_description_sources import PythonLaunchDescriptionSource as load_python_launch_file
from launch.actions import IncludeLaunchDescription as include_another_launch_file 
from launch.actions import DeclareLaunchArgument as create_input_argument
from launch.substitutions import LaunchConfiguration as get_set_argument_val
#from launch.substitutions import PathJoinSubstitution as concatenate_strings


def generate_launch_description(): 
    ####################################################################################
    # get paths for gazebo simulator
    gazebo_package_dir = get_package_share_directory('ros_gz_sim')
    project_dir = get_package_share_directory('multi-robot-simulations')
    ####################################################################################


    ####################################################################################
    # create all arguments for this launch file
    world_name_arg = create_input_argument('world', default_value='worlds/low-end/empty.sdf')
    # setup all configurations in it so you can grab the values when launching the nodes
    get_set_argument_val('world')
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

    # husky spawn path
    husky_launch_path = os.path.join(project_dir, 'launch', 'gazebo_spawn_husky_launch.py')
    spawn_husky_1 = include_another_launch_file(
        load_python_launch_file(husky_launch_path),
        launch_arguments={
            'namespace': 'robot_1',
            'x': '0.0',
            'y': '0.0',
            'z': '0.15'
        }.items()
    )

    spawn_husky_2 = include_another_launch_file(
        load_python_launch_file(husky_launch_path),
        launch_arguments={
            'namespace': 'robot_2',
            'x': '-1.0',
            'y': '0.0',
            'z': '0.15'
        }.items()
    )

    x2_launch_path = os.path.join(project_dir, 'launch', 'gazebo_spawn_x2_launch.py')
    spawn_x2_1 = include_another_launch_file(
        load_python_launch_file(x2_launch_path),
        launch_arguments={
            'namespace': 'robot_3',
            'x': '0.0',
            'y': '1.0',
            'z': '0.15'
        }.items()
    )

    spawn_x2_2 = include_another_launch_file(
        load_python_launch_file(x2_launch_path),
        launch_arguments={
            'namespace': 'robot_4',
            'x': '-1.0',
            'y': '1.0',
            'z': '0.15'
        }.items()
    )

    x4_drone_launch_path = os.path.join(project_dir, 'launch', 'gazebo_spawn_drone_x4_launch.py')
    spawn_x4_drone_1 = include_another_launch_file(
        load_python_launch_file(x4_drone_launch_path),
        launch_arguments={
            'namespace': 'robot_5',
            'x': '0.0',
            'y': '2.0',
            'z': '0.15'
        }.items()
    )   

    spawn_x4_drone_2 = include_another_launch_file(
        load_python_launch_file(x4_drone_launch_path),
        launch_arguments={
            'namespace': 'robot_6',
            'x': '-1.0',
            'y': '2.0',
            'z': '0.15'
        }.items()
    )   

    return LaunchDescription([
        world_name_arg,
        gz_sim_launch,
        spawn_husky_1,
        spawn_husky_2,
        spawn_x2_1,
        spawn_x2_2,
        spawn_x4_drone_1,
        spawn_x4_drone_2
    ])