# Configuring your Working Environment

This tutorial was tested in June 19, 2024 in several different desktop computers and laptops from different ages and it worked at first attempet. It assumes that you have previous experience installing an operational systems in your personal computer and have an understanding regarding tools, such as git, curl, bash, and tmux, specifically how to run commands in a terminal.

**Dificulty: medium**

- [Configuring your Working Environment](#configuring-your-working-environment)
  - [Summary](#summary)
  - [Steps to Install Ubuntu 24.04](#steps-to-install-ubuntu-2404)
  - [Steps to Install ROS 2 Jazzy Jalisco](#steps-to-install-ros-2-jazzy-jalisco)
  - [Steps to Install and Configure Tmux and Git](#steps-to-install-and-configure-tmux-and-git)
  - [Next Steps](#next-steps)

## [Summary](#summary-link)

All simulations in this repository are intended to run on native Ubuntu 24.04. While many students and professionals might prefer to use Docker and install ROS in a container, I believe that the additional system architecture layer introduced by Docker is detrimental to understanding, mainly because ROS itself can be very complicated for the inexperienced explorer, as I’ve explained [here](docs/motivation). Another reason is that when working with mobile robots, we often need to visualize what is happening, and unfortunately, there is no easy way to make the graphics card driver communicate with the native system if they are different (e.g., a container running Ubuntu 20.04 with a host machine running Ubuntu 24.04 with different glibc libraries), which results in very clunky simulations. In summary I've divided this tutorial in three parts:

- **Installing Ubuntu:** The installation of an opperational system is a little bit tricky, however, there are plenty of tutorials online teaching you how to do it. For example, there is a great tutorial at [ubuntu website](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview)

- **Installing ROS 2 Jazzy Jalisco:** After installing ubuntu, you will need to install ROS 2 and all its dependencies. For this purpose, I recommend using Ubuntu's package manager, because it is easier and the chances of breaking something in your system are minimal. The steps presented here are a compilation of the ones from the original ROS 2 Jazzy documentation that can be accessed through this [link](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html).

- **Configure your Working Environment with Tmux and Git:** At this point you are ready to start developing, however, you might fall into an organizational trap like many professionals and students, which is to have too much information to handle in an unstructured way. Since ROS is composed by many programs and each one has its own output, I often see people opening several terminals and getting lost in what is going on, turn the learning or development process way more complicated than it should. Therefore, I believe that having a clean working environment, where you understand exactly what is going on, turn your understanding and development more easy. To better organize your working environment, I advise you to install a terminal multiplexer, such as [tmux](https://github.com/tmux/tmux/wiki).

## [Steps to Install Ubuntu 24.04](#ubuntu-2404)

In summary, you would need to follow these steps:

1. Download Ubuntu 24.04 image from [here](https://releases.ubuntu.com/noble/ubuntu-24.04-desktop-amd64.iso)
2. Use a USB Startup Disk Creator, such as [Rufus](https://rufus.ie/en/) to create a bootable device using the image downloaded from step 1.
3. Restart your computer, boot from the pendrive, and follow the instructions.
4. After finishing the installation, boot in Ubuntu 24.04 and login with your user name and password.

## [Steps to Install ROS 2 Jazzy Jalisco](#ros-2-jazzy-jalisco)

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

6. Add an entry at the start of your .bashrc file to make your system be able to see where ROS programs are installed.

    ```text
    source /opt/ros/jazzy/setup.bash
    ```

    the starting of your .bashrc file should look something like this

    ```bash
    # ~/.bashrc: executed by bash(1) for non-login shells.
    # see /usr/share/doc/bash/examples/startup-files (in the package bash-doc)
    # for examples

    source /opt/ros/jazzy/setup.bash

    # If not running interactively, don't do anything
    case $- in
        *i*) ;;
          *) return;;
    esac
    ```

## [Steps to Install and Configure Tmux and Git](#tmux-basic-setup)

In summary, you need to follow these steps:

1. Install Git hub program to be able to clone and push into remote repositories.

    ```text
    sudo apt install git
    ```

2. Install tmux program with the following command.

    ```text
    sudo apt install tmux
    ```

2. Install tmux plugin manager for a better experience.

    ```text
    git clone https://github.com/tmux-plugins/tpm ~/.tmux/plugins/tpm
    ```

3. Copy the following configuration into a file named ```~/.tmux.conf```

    ```bash
    # List of plugins
    set -g @plugin 'tmux-plugins/tpm'
    set -g @plugin 'tmux-plugins/tmux-sensible'
    set -g mouse on

    # Initialize TMUX plugin manager (keep this line at the very bottom of tmux.conf)
    run '~/.tmux/plugins/tpm/tpm'
    ```

## [Next Steps](#next-steps)

  Now your environment should be ready to run the simulations from this repository and also to help you starting your journey.