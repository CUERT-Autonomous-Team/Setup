
# Setup

We will be developing our code using **ROS** on **Ubuntu** for **Carla**.

Don't hesitate to ask for help, if you face any problems with the installation.

## Ubuntu

Download **Ubuntu 20.04 LTS (Focal Fossa)** from [here](https://releases.ubuntu.com/20.04/).

***Install** the desktop image

Follow this [video](https://www.youtube.com/watch?v=GXxTxBPKecQ) to install Ubuntu alongside Windows as a dual boot.

***Allocate** 70GB of space or more for Ubuntu

Make yourself familiar with Ubuntu and install **VSCode** and **Discord** from Ubuntu software store.

Start a terminal and run:

```bash

sudo apt update

sudo apt upgrade

```

## ROS

Follow this [guide](http://wiki.ros.org/noetic/Installation/Ubuntu) to install **ROS 1 Noetic**.

***Install** the full desktop version (aka. ros-noetic-desktop-full)

***Install** important ros packages:

```bash

  sudo apt install python3-catkin-tools python3-rosdep \

  ros-noetic-joy ros-noetic-teleop-twist-joy \

  ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \

  ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \

  ros-noetic-rosserial-python ros-noetic-rosserial-client \

  ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \

  ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \

  ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \

  ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers

```

***Run** `sudo rosdep init`

Test ROS by running the following command:

```bash

roscore

```

## Turtlebot3

Run the following command to install Turtlebot3:

```bash

sudo apt install ros-noetic-dynamixel-sdk
sudo apt install ros-noetic-turtlebot3-msgs
sudo apt install ros-noetic-turtlebot3
sudo apt install ros-noetic-turtlebot3-simulations

```

Test Turtlebot3 by running the following commands:

```bash

export TURTLEBOT3_MODEL=burger

roslaunch turtlebot3_gazebo turtlebot3_world.launch

```

## Carla

Download **Carla 0.9.11** from [here](https://github.com/carla-simulator/carla/releases/tag/0.9.11/).

***Install** CARLA_0.9.11.tar.gz for Ubuntu

***Extract** the compressed file

Test Carla by running the following command:

```bash

./CarlaUE4.sh -quality-level=Low-windowed-fps-novsync-resx=360-resy=240

```

Install important python packages:

```bash

pip install --user pygame numpy

```

## Carla/ROS Bridge

Download **Carla/ROS Bridge 0.9.11** from [here](https://github.com/carla-simulator/ros-bridge/releases/tag/0.9.11).

***Install** Source code (tar.gz)

***Extract** the compressed file

Enclose the extracted folders in two parent folders (e.g. `carla-ros-bridge/src/ros-bridge/<extracted-folders>`) and run below commands from `carla-ros-bridge` folder:

```bash

rosdep update

rosdep install--from-pathssrc--ignore-src-r

catkin build

```

Install important python packages:

```bash

sudo apt install python-is-python3

sudo apt install python3-opencv

```

Test Carla/ROS Bridge by running the following commands:

```bash

export CARLA_ROOT=<path-to-carla>

export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla/dist/carla-0.9.11-py3.7-linux-x86_64.egg:$CARLA_ROOT/PythonAPI/carla

source devel/setup.bash

roslaunch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch

```

## VSCode

Run below commands to install helpful extensions:

```bash

code--install-extensionaaron-bond.better-comments

code--install-extensionakamud.vscode-theme-onedark

code--install-extensionchristian-kohler.path-intellisense

code--install-extensioncschlosser.doxdocgen

code--install-extensionDavidAnson.vscode-markdownlint

code--install-extensioneamodio.gitlens

code--install-extensionhbenl.vscode-test-explorer

code--install-extensionIronGeek.vscode-env

code--install-extensionjeff-hykin.better-cpp-syntax

code--install-extensionmatepek.vscode-catch2-test-adapter

code--install-extensionms-azuretools.vscode-docker

code--install-extensionms-python.python

code--install-extensionms-vscode-remote.remote-containers

code--install-extensionms-vscode-remote.remote-ssh

code--install-extensionms-vscode-remote.remote-ssh-edit

code--install-extensionms-vscode-remote.remote-wsl

code--install-extensionms-vscode-remote.vscode-remote-extensionpack

code--install-extensionms-vscode.cmake-tools

code--install-extensionms-vscode.cpptools

code--install-extensionms-vscode.test-adapter-converter

code--install-extensionms-vscode.vs-keybindings

code--install-extensionPKief.material-icon-theme

code--install-extensionredhat.vscode-xml

code--install-extensionstreetsidesoftware.code-spell-checker

code--install-extensiontwxs.cmake

code--install-extensionusernamehw.errorlens

code--install-extensionyzhang.markdown-all-in-one

```
