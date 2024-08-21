# Table of Contents

- [Table of Contents](#table-of-contents)
  - [Working Environment](#working-environment)
  - [Disclaimer About Docker](#disclaimer-about-docker)
  - [Steps to Install Ubuntu 24.04](#steps-to-install-ubuntu-2404)
  - [Steps to Install ROS 2 Jazzy Jalisco](#steps-to-install-ros-2-jazzy-jalisco)
  - [Steps to Install and Configure Tmux and Git](#steps-to-install-and-configure-tmux-and-git)
  - [Next Steps](#next-steps)

## [Working Environment](#working-environment)

This workspace was tested on June 19, 2024, on several different desktop computers and laptops of varying ages, and it worked on the first attempt. It assumes that you have previous experience installing operating systems on your personal computer and understand tools such as git, curl, bash, and tmux, specifically how to run commands in a terminal.

## [Disclaimer About Docker](#disclaimer-about-docker)

All simulations in this workspace are intended to run on native Ubuntu 24.04. While many students and professionals might prefer to use Docker and install ROS in a container, I believe that the additional system architecture layer introduced by Docker is detrimental to understanding, mainly because ROS itself can be very complicated for the inexperienced explorer, as I’ve explained here. Another reason is that when working with mobile robots, we often need to visualize what is happening. Unfortunately, there is no easy way to make the graphics card driver communicate with the native system if they are different (e.g., a container running Ubuntu 20.04 with a host machine running Ubuntu 24.04 with different glibc libraries), which results in very clunky simulations.

In summary, I've divided this tutorial into three parts:

- **Steps to Installing Ubuntu 24.04:** Installing an operating system can be a bit tricky, but there are plenty of tutorials online to guide you. For example, there's a great tutorial on the Ubuntu website.

- **Steps to Install ROS 2 Jazzy Jalisco:** After installing Ubuntu, you will need to install ROS 2 and all its dependencies. For this purpose, I recommend using Ubuntu's package manager because it is easier and the chances of breaking something in your system are minimal. The steps presented here are a compilation from the original ROS 2 Jazzy documentation, which can be accessed through this link.

- **Steps to Install and Configure Tmux and Git:** At this point, you are ready to start developing. However, you might fall into an organizational trap, like many professionals and students, by having too much information to handle in an unstructured way. Since ROS is composed of many programs, each with its own output, I often see people opening several terminals and getting lost in what is going on, making the learning or development process more complicated than it should be. Therefore, I believe that having a clean working environment where you understand exactly what is going on makes your understanding and development easier. To better organize your working environment, I advise you to install a terminal multiplexer, such as tmux.

## [Steps to Install Ubuntu 24.04](#steps-to-install-ubuntu-2404)

In summary, you would need to follow these steps:

1. Download Ubuntu 24.04 image from [here](https://releases.ubuntu.com/noble/ubuntu-24.04-desktop-amd64.iso)
2. Use a USB Startup Disk Creator, such as [Rufus](https://rufus.ie/en/) to create a bootable device using the image downloaded from step 1.
3. Restart your computer, boot from the pendrive, and follow the instructions.
4. After finishing the installation, boot in Ubuntu 24.04 and login with your user name and password.

## [Steps to Install ROS 2 Jazzy Jalisco](#steps-to-install-ros-2-jazzy-jalisco)

In summary, you need to follow these steps:

1. After loggin in Ubuntu, open a terminal.

2. At the terminal install the following package and add an entry pointing to ROS Jazzy repsitories at your sources list with the following commands.

    ```text
    sudo apt install software-properties-common
    sudo add-apt-repository universe
    sudo apt update && sudo apt install curl -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    ```

3. Install all development dependencies to work with ROS, such as Python 3, with the following commands.

    ```text
    sudo apt update
    sudo apt install ros-dev-tools
    ```

4. Install ROS 2 jazzy with the following command. In particullar, I preffer installing the full package for research purposes to reduce the chances of unmet dependencies when testing and working with robots.

    ```text
    sudo apt install ros-jazzy-desktop-full
    ```

5. For some reason, even after installing the full version of ROS 2, it does not install an important package that allow you to represent robots' joint states in a simulation. Therefore, you should install the joint-state-publisher package with the following command.

    ```text
    sudo apt install ros-jazzy-joint-state-publisher
    ```

6. Add an entry at the start of your .bashrc file to make your system be able to see where ROS programs are installed and the resources provided in this workspace.

    ```text
    sed -i '1i export GZ_SIM_RESOURCE_PATH = PATH_TO_RESOURCES' ~/.bashrc
    sed -i '1i source /opt/ros/jazzy/setup.bash' ~/.bashrc
    ```

    the starting of your .bashrc file should look something like this

    ```bash
    source /opt/ros/jazzy/setup.bash
    export GZ_SIM_RESOURCE_PATH = PATH_TO_RESOURCES
    # ~/.bashrc: executed by bash(1) for non-login shells.
    # see /usr/share/doc/bash/examples/startup-files (in the package bash-doc)
    # for examples

    # If not running interactively, don't do anything
    case $- in
        *i*) ;;
          *) return;;
    esac
    ```

    In this case, your MUST replace ```PATH_TO_RESORRCES``` with the path of the ```Multi-robot-Sandbox/gazebo_resources``` from this workspace. This allows both, Ignition Gazebo and RViz to see the available resources
    and load the models accordingly. Specifically, I`ve configured the launch files to dynamically set this path inside the ```.sdf``` model files so both programs can handle them appropriately.

## [Steps to Install and Configure Tmux and Git](#steps-to-install-and-configure-tmux-and-git)

In summary, you need to follow these steps:

1. Install Git hub program to be able to clone and push into remote repositories.

    ```text
    sudo apt install git
    ```

2. Install tmux program with the following command.

    ```text
    sudo apt install tmux
    ```

3. Install tmux plugin manager for a better experience.

    ```text
    git clone https://github.com/tmux-plugins/tpm ~/.tmux/plugins/tpm
    ```

4. Copy the following configuration into a file named ```~/.tmux.conf```

    ```bash
    set -g @plugin 'o0th/tmux-nova'

    set -g @nova-nerdfonts true
    set -g @nova-nerdfonts-left 
    set -g @nova-nerdfonts-right 

    set -g @nova-segment-mode "#{?client_prefix,Ω,ω}"
    set -g @nova-segment-mode-colors "#50fa7b #282a36"

    set -g @nova-segment-whoami "#(whoami)@#h"
    set -g @nova-segment-whoami-colors "#50fa7b #282a36"

    set -g @nova-pane "#I#{?pane_in_mode,  #{pane_mode},}  #W"

    set -g @nova-rows 0
    set -g @nova-segments-0-left "mode"
    set -g @nova-segments-0-right "whoami"

    # List of plugins
    set -g @plugin 'tmux-plugins/tpm'
    set -g @plugin 'tmux-plugins/tmux-sensible'
    set -g mouse on

    # Initialize TMUX plugin manager (keep this line at the very bottom of tmux.conf)
    run '~/.tmux/plugins/tpm/tpm'
    ```

5. Open tmux by typing ```tmux``` in a terminal.

6. Enter command mode with ```ctr+b```.
   
7. Install plugins with ```shift+i``` while in command mode.

8. Install a ```patched font``` from [here](https://github.com/ryanoasis/nerd-fonts/releases/download/v3.2.1/AurulentSansMono.zip) and make it default in your terminal. I also recommend turning off the ```Use colors from system theme```.

## [Next Steps](#next-steps)

  Now your environment should be ready to run the simulations from this repository and also to help you starting your journey. [Click here](usage.md) for the next steps.
