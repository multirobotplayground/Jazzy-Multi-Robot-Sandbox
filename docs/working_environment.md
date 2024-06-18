# Configuring your Working Environment

- [Configuring your Working Environment](#configuring-your-working-environment)
  - [Ubuntu 24.04](#ubuntu-2404)
  - [ROS 2 Jazzy Jalisco](#ros-2-jazzy-jalisco)
  - [Tmux Basic Setup](#tmux-basic-setup)

## [Ubuntu 24.04](#ubuntu-2404)

All simulations in this repository are intended to run on native Ubuntu 24.04. While many students and professionals might prefer to use Docker and install ROS in a container, I believe that the additional system architecture layer introduced by Docker is detrimental to understanding, mainly because ROS itself can be very complicated for the inexperienced explorer, as I’ve explained [here](docs/motivation). Another reason is that when working with mobile robots, we often need to visualize what is happening, and unfortunately, there is no easy way to make the graphics card driver communicate with the native system if they are different (e.g., a container running Ubuntu 20.04 with a host machine running Ubuntu 24.04 with different glibc libraries), which results in very clunky simulations.

## [ROS 2 Jazzy Jalisco](#ros-2-jazzy-jalisco)



## [Tmux Basic Setup](#tmux-basic-setup)