# Setup

We will be developing our code using **ROS** on **Ubuntu** for **Carla**.

Don't hesitate to ask for help, if you face any problems with the installation.

## Ubuntu

Download **Ubuntu 20.04 LTS (Focal Fossa)** from [here](https://releases.ubuntu.com/20.04/).

* **Install** the desktop image

Follow this [video](https://www.youtube.com/watch?v=GXxTxBPKecQ) to install Ubuntu alongside Windows as a dual boot.

* **Allocate** 40GB of space or more for Ubuntu

Make yourself familiar with Ubuntu and install **VSCode** and **Discord** from Ubuntu software store.

Start a terminal and run:

```bash
sudo apt update
sudo apt upgrade
```

## ROS

Follow this [guide](http://wiki.ros.org/noetic/Installation/Ubuntu) to install **ROS 1 Noetic**.

* **Install** the full desktop version (aka. ros-noetic-desktop-full)
* **Install** important ros packages:

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

* **Run** `sudo rosdep init`

Test ROS by running the following command:

```bash
roscore
```

## Turtlebot3

Run the following command to install Turtlebot3:

```bash
sudo apt install ros-noetic-dynamixel-sdk ros-noetic-turtlebot3-msgs \
ros-noetic-turtlebot3 ros-noetic-turtlebot3-simulations
```

Test Turtlebot3 by running the following commands:

```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

## Carla

Download **Carla 0.9.11** from [here](https://github.com/carla-simulator/carla/releases/tag/0.9.11/).

* **Install** CARLA_0.9.11.tar.gz for Ubuntu
* **Extract** the compressed file

Test Carla by running the following command:

```bash
./CarlaUE4.sh -quality-level=Low -windowed -fps -novsync -resx=360 -resy=240
```

Install important python packages:

```bash
pip install --user pygame numpy
```

## Carla/ROS Bridge

Download **Carla/ROS Bridge 0.9.11** from [here](https://github.com/carla-simulator/ros-bridge/releases/tag/0.9.11).

* **Install** Source code (tar.gz)
* **Extract** the compressed file

Enclose the extracted folders in two parent folders (e.g. `carla-ros-bridge/src/ros-bridge/<extracted-folders>`) and run below commands from `carla-ros-bridge` folder:

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r
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
code --install-extension aaron-bond.better-comments
code --install-extension akamud.vscode-theme-onedark
code --install-extension christian-kohler.path-intellisense
code --install-extension cschlosser.doxdocgen
code --install-extension DavidAnson.vscode-markdownlint
code --install-extension eamodio.gitlens
code --install-extension hbenl.vscode-test-explorer
code --install-extension IronGeek.vscode-env
code --install-extension jeff-hykin.better-cpp-syntax
code --install-extension matepek.vscode-catch2-test-adapter
code --install-extension ms-azuretools.vscode-docker
code --install-extension ms-python.python
code --install-extension ms-vscode-remote.remote-containers
code --install-extension ms-vscode-remote.remote-ssh
code --install-extension ms-vscode-remote.remote-ssh-edit
code --install-extension ms-vscode-remote.remote-wsl
code --install-extension ms-vscode-remote.vscode-remote-extensionpack
code --install-extension ms-vscode.cmake-tools
code --install-extension ms-vscode.cpptools
code --install-extension ms-vscode.test-adapter-converter
code --install-extension ms-vscode.vs-keybindings
code --install-extension PKief.material-icon-theme
code --install-extension redhat.vscode-xml
code --install-extension streetsidesoftware.code-spell-checker
code --install-extension twxs.cmake
code --install-extension usernamehw.errorlens
code --install-extension yzhang.markdown-all-in-one
```
