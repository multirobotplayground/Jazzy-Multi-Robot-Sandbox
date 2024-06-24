# Table of Contents

- [Table of Contents](#table-of-contents)
  - [Usage](#usage)
  - [Main Launch File](#main-launch-file)
  - [Example Launch Files](#example-launch-files)
  - [Robot Spawn Launch Files](#robot-spawn-launch-files)
  - [Rviz Configuration Files](#rviz-configuration-files)
  - [Communicating with the Robots](#communicating-with-the-robots)
  - [Adding More Robots](#adding-more-robots)
  - [Extra](#extra)

## [Usage](#usage)

Before running the simulation, make sure you followed the steps I've described in the [working environment](working_environment.md) and that Ubuntu 24.04 and ROS 2 Jazzy Jalisco are properly installed and follow these steps.

1. Open a terminal or use ```tmux```.
2. Enter the ```Multi-robot-Sandbox``` folder in the terminal I`ve oppened.
3. Compile the package and source its contents so your system can locate it.

    ```bash
    colcon build
    source install/local_setup.bash
    ```

4. Run the main simulation launch file.

    ```bash
    ros2 launch multi-robot-simulations gazebo_multi_robot_bringup_launch.py
    ```

5. Click on the play button on the botton left of the simulator.

If everything worked correctly, you should see the following scene:

![Ignition](images/ign.png "ignition")

I encourage you to explore the scene tree and also to open the Rviz configuration files I've setup for this launch file.

## [Main Launch File](#main-launch-file)

- [gazebo_multi_robot_bringup_launch.py](../src/multi-robot-simulations/launch/gazebo_multi_robot_bringup_launch.py)

## [Example Launch Files](#example-launch-files)

- [gazebo_single_robot_gz_bridge_config_file_launch.py](../src/multi-robot-simulations/launch/gazebo_single_robot_gz_bridge_config_file_launch.py)
- [gazebo_single_robot_gz_bridge_no_config_file_launch.py](../src/multi-robot-simulations/launch/gazebo_single_robot_gz_bridge_no_config_file_launch.py)

## [Robot Spawn Launch Files](#robot-spawn-launch-files)

- [gazebo_spawn_drone_x4_launch.py](../src/multi-robot-simulations/launch/gazebo_spawn_drone_x4_launch.py)
- [gazebo_spawn_husky_launch.py](../src/multi-robot-simulations/launch/gazebo_spawn_husky_launch.py)
- [gazebo_spawn_x2_launch.py](../src/multi-robot-simulations/launch/gazebo_spawn_x2_launch.py)

## [Rviz Configuration Files](#rviz-configuration-files)

There is one configuration file for each robot I've configured in the main launch file. They are located at ```src/multi-robot-simulations/config/rviz``` folder.

To run them, do the following.

1. Ensure that the simulation is running as [described](#usage).
2. Open a new terminal or use ```tmux```.
3. Open RViz 2 with the following command in your new tmux partition.

    ```bash
    rviz2
    ```

4. Go to the top menu of RViz and load one of the configuration files provided.

If everything works correctly, you should see something similar to this.

![Rviz](images/rviz.png "Rviz")

## [Communicating with the Robots](#communicating-with-the-robots)

To communicate with the robots through ROS 2, you can check the available topics by running the following.

1. Ensure that the simulation is running as [described](#usage).
2. Open a new terminal or use ```tmux```.
3. Run the following command on your new tmux partition.

    ```bash
    ros2 topic list
    ```

If everything was done correctly, you should see the following topics in your terminal

```bash
/clock
/parameter_events
/robot_1/cmd_vel
/robot_1/imu
/robot_1/joint_states
/robot_1/lidar/points
/robot_1/lidar/scan
/robot_1/odometry
/robot_1/pose
/robot_1/robot_description
/robot_2/cmd_vel
/robot_2/imu
/robot_2/lidar/points
/robot_2/lidar/scan
/robot_2/odometry
/robot_2/pose
/robot_2/robot_description
/robot_3/cmd_vel
/robot_3/imu
/robot_3/joint_states
/robot_3/lidar/points
/robot_3/lidar/scan
/robot_3/odometry
/robot_3/pose
/robot_3/robot_description
/robot_4/cmd_vel
/robot_4/imu
/robot_4/joint_states
/robot_4/lidar/points
/robot_4/lidar/scan
/robot_4/odometry
/robot_4/pose
/robot_4/robot_description
/robot_5/camera/camera_info
/robot_5/camera/depth_image
/robot_5/camera/image
/robot_5/camera/points
/robot_5/cmd_vel
/robot_5/imu
/robot_5/joint_states
/robot_5/odometry
/robot_5/pose
/robot_5/robot_description
/robot_6/camera/camera_info
/robot_6/camera/depth_image
/robot_6/camera/image
/robot_6/camera/points
/robot_6/cmd_vel
/robot_6/imu
/robot_6/joint_states
/robot_6/odometry
/robot_6/pose
/robot_6/robot_description
/rosout
/tf
/tf_static
```

Since robots are properly configured with their namepsace, you should be able to control and visualize them apropriately.

## [Adding More Robots](#adding-more-robots)

To add more robots simply run one of the robot spawn launch files I've configured specifying a namespace different from the ones from the robots already running. For example, if you wanna add an extra Husky under the namespace robot_7 at x=3, y,3, z=0.5, run the following command in a new terminal or with ```tmux```.

```bash
ros2 launch multi-robot-simulations gazebo_spawn_husky_launch.py namespace:=robot_7 x:=3.0 y:=3.0 z:=0.5
```

if the robot is loaded correctly, the launch file should handle the transformations and parametrization stuff for you, publishing the correct topics under the specified namespace. You should see the following in your Ignition Gazebo.

![new_robot](images/new_robot.png "New robot")

You should also see the following extra topics available to communicate with your robot if you run the command ```ros2 topic list``` in a terminal.

```bash
/robot_7/cmd_vel
/robot_7/imu
/robot_7/joint_states
/robot_7/lidar/points
/robot_7/lidar/scan
/robot_7/odometry
/robot_7/pose
/robot_7/robot_description
```

## [Extra](#extra)

ROS 2 is yet an experimental environment that is constantly evolving due to the great community, it has much to improve, however the tools it provides suffice for most use cases. Some hacks had to be made to work with the parameterized launch files and robot configurations to make them integrate seamless with Ignition Gazebo and RViz and to work in a **multi-robot** setting. 

Finally, I hope that this could make you skip the stuff that some people might find booring! You can add and use as many robots your computer is able to handle and start doing your stuff, so keep playing around! :)
