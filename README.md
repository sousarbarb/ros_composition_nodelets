# [ros_composition_nodelets](https://github.com/sousarbarb/ros_composition_nodelets)

Repository to exemplify the Intra Process Communication (IPC) implementation of
the [Robot Operating System (ROS)](https://ros.org/).
ROS 1 has the [Nodelet](http://wiki.ros.org/nodelet/) to run multiple algorithms
in the same process with zero copy transport between algorithms.
The equivalent technology in ROS 2 is
[Node Composition](https://docs.ros.org/en/foxy/Concepts/About-Composition.html)
as a unified API, in constrast to Nodelets.


## Setup

### ClangFormat

**Installation**

```sh
sudo apt update
sudo apt install -y clang-format
```

**Format Configuration File**
```sh
# If you want to create a format configuration file for the first time...
# https://clang.llvm.org/docs/ClangFormat.html
clang-format --style=<llvm|google|chromium|mozilla|webKit> --dump-config > .clang-format
```

**Visual Studio Code**

1. Install the C/C++ extension (ms-vscode.cpptools)
2. View > Command Pallet... (`Ctrl+Shift+P`)
   - Preferences: Open User Settings (JSON)
3. Paste the following in your user settings JSON file
   ```json
   {
     "editor.formatOnSave": true,
     "[cpp]": {
       "editor.defaultFormatter": "ms-vscode.cpptools"
     },
     "C_Cpp.clang_format_style": "file",
     "C_Cpp.formatting": "clangFormat"
   }
   ```

### Robot Operating System (ROS)

**ROS 1**

```sh
sudo apt update
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl -y
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-<ROS1_DISTRO>-desktop-full -y
source /opt/ros/<ROS1_DISTRO>/setup.bash
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y
sudo rosdep init
rosdep update
```

See [https://wiki.ros.org/Installation/Ubuntu](https://wiki.ros.org/Installation/Ubuntu)
for further details on the ROS 1 installation instructions.

In order to test if the installation was successful, run the following code:

```sh
source /opt/ros/<ROS1_DISTRO>/setup.bash
roslaunch roscpp_tutorials talker_listener.launch
```

**ROS 2**

```sh
sudo apt update && sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale
sudo apt install software-properties-common -y
sudo add-apt-repository universe -y
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-<ROS2_DISTRO>-desktop python3-argcomplete ros-dev-tools -y
```

See [https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
for further details on the ROS 2 installation instructions.

In order to test if the installation was successful, run the following code:

```sh
source /opt/ros/<ROS2_DISTRO>/setup.bash
ros2 launch demo_nodes_cpp talker_listener.launch.xml
```

## Usage

### Repository

```sh
mkdir ~/ros_ws/src -p
cd ~/ros_ws/src
git clone git@github.com:sousarbarb/ros_composition_nodelets.git
```

### Build

**ROS 1**

```sh
source /opt/<ROS1_DISTRO>/setup.bash

cd ~/ros_ws
catkin_make --force-cmake --cmake-args -DCMAKE_BUILD_TYPE=Release

source devel/setup.bash
```

**ROS 2**

```sh
source /opt/<ROS2_DISTRO>/setup.bash

cd ~/ros_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --event-handlers status+ console_direct+ console_start_end+

source install/setup.bash
```

### Launch: ROS 1

**Independent Nodes**

```sh
$ roslaunch ros_composition_nodelets ros1_independent.launch
...
process[rosout-1]: started with pid [112535]
started core service [/rosout]
process[pub0-2]: started with pid [112538]
process[sub1-3]: started with pid [112540]
process[sub2-4]: started with pid [112544]
[ INFO] [1741980303.213399983]: Publishing: 'Hello, world! (0) (pub ptr: 0x7fc11c000eb0)'
[ INFO] [1741980303.214637166]: I heard: 'Hello, world! (0) (pub ptr: 0x7fc11c000eb0)' (sub ptr: 0x7f024c000eb0)
[ INFO] [1741980303.214714341]: I heard: 'Hello, world! (0) (pub ptr: 0x7fc11c000eb0)' (sub ptr: 0x7fc384000eb0)
[ INFO] [1741980304.213265894]: Publishing: 'Hello, world! (1) (pub ptr: 0x7fc11c001180)'
[ INFO] [1741980304.213650584]: I heard: 'Hello, world! (1) (pub ptr: 0x7fc11c001180)' (sub ptr: 0x7f024c000eb0)
[ INFO] [1741980304.213679080]: I heard: 'Hello, world! (1) (pub ptr: 0x7fc11c001180)' (sub ptr: 0x7fc384000eb0)
[ INFO] [1741980305.213376403]: Publishing: 'Hello, world! (2) (pub ptr: 0x7fc11c001180)'
[ INFO] [1741980305.214003305]: I heard: 'Hello, world! (2) (pub ptr: 0x7fc11c001180)' (sub ptr: 0x7fc384000eb0)
[ INFO] [1741980305.214011616]: I heard: 'Hello, world! (2) (pub ptr: 0x7fc11c001180)' (sub ptr: 0x7f024c000eb0)
[ INFO] [1741980306.213190094]: Publishing: 'Hello, world! (3) (pub ptr: 0x7fc11c001180)'
[ INFO] [1741980306.213504732]: I heard: 'Hello, world! (3) (pub ptr: 0x7fc11c001180)' (sub ptr: 0x7f024c000eb0)
[ INFO] [1741980306.213514231]: I heard: 'Hello, world! (3) (pub ptr: 0x7fc11c001180)' (sub ptr: 0x7fc384000eb0)
```

**Nodelets**

```sh
$ roslaunch ros_composition_nodelets ros1_nodelets.launch
...
[ INFO] [1741980404.844864730]: Initializing nodelet with 32 worker threads.
[ INFO] [1741980405.867858222]: Publishing: 'Hello, world! (0) (pub ptr: 0x7f8888000eb0)'
[ INFO] [1741980405.867959144]: I heard: 'Hello, world! (0) (pub ptr: 0x7f8888000eb0)' (sub ptr: 0x7f8888000eb0)
[ INFO] [1741980405.868042256]: I heard: 'Hello, world! (0) (pub ptr: 0x7f8888000eb0)' (sub ptr: 0x7f8888000eb0)
[ INFO] [1741980406.867313164]: Publishing: 'Hello, world! (1) (pub ptr: 0x7f88880015b0)'
[ INFO] [1741980406.867410524]: I heard: 'Hello, world! (1) (pub ptr: 0x7f88880015b0)' (sub ptr: 0x7f88880015b0)
[ INFO] [1741980406.867510258]: I heard: 'Hello, world! (1) (pub ptr: 0x7f88880015b0)' (sub ptr: 0x7f88880015b0)
[ INFO] [1741980407.867505438]: Publishing: 'Hello, world! (2) (pub ptr: 0x7f8888001600)'
[ INFO] [1741980407.867663352]: I heard: 'Hello, world! (2) (pub ptr: 0x7f8888001600)' (sub ptr: 0x7f8888001600)
[ INFO] [1741980407.867752400]: I heard: 'Hello, world! (2) (pub ptr: 0x7f8888001600)' (sub ptr: 0x7f8888001600)
[ INFO] [1741980408.867863947]: Publishing: 'Hello, world! (3) (pub ptr: 0x7f8888002a30)'
[ INFO] [1741980408.868119220]: I heard: 'Hello, world! (3) (pub ptr: 0x7f8888002a30)' (sub ptr: 0x7f8888002a30)
[ INFO] [1741980408.868216580]: I heard: 'Hello, world! (3) (pub ptr: 0x7f8888002a30)' (sub ptr: 0x7f8888002a30)
```

### Launch: ROS 2

**Independent Nodes**

```sh
$ ros2 launch ros_composition_nodelets ros2_independent.launch.xml
...
[pub2-1] [INFO] [1741980555.098423081] [pub0]: Publishing: 'Hello, world! (0) (pub ptr: 0x556d611f4c90)'
[sub2-3] [INFO] [1741980555.099074919] [sub2]: I heard: 'Hello, world! (0) (pub ptr: 0x556d611f4c90)' (sub ptr: 0x55c277f23990)
[sub2-2] [INFO] [1741980555.099128348] [sub1]: I heard: 'Hello, world! (0) (pub ptr: 0x556d611f4c90)' (sub ptr: 0x55ed6def3f60)
[pub2-1] [INFO] [1741980556.098393304] [pub0]: Publishing: 'Hello, world! (1) (pub ptr: 0x556d611f4c90)'
[sub2-2] [INFO] [1741980556.098821926] [sub1]: I heard: 'Hello, world! (1) (pub ptr: 0x556d611f4c90)' (sub ptr: 0x55ed6def3f60)
[sub2-3] [INFO] [1741980556.098837361] [sub2]: I heard: 'Hello, world! (1) (pub ptr: 0x556d611f4c90)' (sub ptr: 0x55c277f23990)
[pub2-1] [INFO] [1741980557.098375407] [pub0]: Publishing: 'Hello, world! (2) (pub ptr: 0x556d611f4c90)'
[sub2-3] [INFO] [1741980557.098754162] [sub2]: I heard: 'Hello, world! (2) (pub ptr: 0x556d611f4c90)' (sub ptr: 0x55c277f23990)
[sub2-2] [INFO] [1741980557.098756536] [sub1]: I heard: 'Hello, world! (2) (pub ptr: 0x556d611f4c90)' (sub ptr: 0x55ed6def3f60)
[pub2-1] [INFO] [1741980558.098412134] [pub0]: Publishing: 'Hello, world! (3) (pub ptr: 0x556d611f4c90)'
[sub2-3] [INFO] [1741980558.098865689] [sub2]: I heard: 'Hello, world! (3) (pub ptr: 0x556d611f4c90)' (sub ptr: 0x55c277f23990)
[sub2-2] [INFO] [1741980558.098865689] [sub1]: I heard: 'Hello, world! (3) (pub ptr: 0x556d611f4c90)' (sub ptr: 0x55ed6def3f60)
```

**Node Composition**

```sh
$ ros2 launch ros_composition_nodelets ros2_components.launch.py
...
[component_container-1] [INFO] [1741980594.616565432] [pub0]: Publishing: 'Hello, world! (0) (pub ptr: 0x55c9f936ddb0)'
[component_container-1] [INFO] [1741980594.616994054] [sub1]: I heard: 'Hello, world! (0) (pub ptr: 0x55c9f936ddb0)' (sub ptr: 0x55c9f936ddb0)
[component_container-1] [INFO] [1741980594.617052233] [sub2]: I heard: 'Hello, world! (0) (pub ptr: 0x55c9f936ddb0)' (sub ptr: 0x55c9f936ddb0)
[component_container-1] [INFO] [1741980595.616474172] [pub0]: Publishing: 'Hello, world! (1) (pub ptr: 0x55c9f936ddb0)'
[component_container-1] [INFO] [1741980595.616753192] [sub1]: I heard: 'Hello, world! (1) (pub ptr: 0x55c9f936ddb0)' (sub ptr: 0x55c9f936ddb0)
[component_container-1] [INFO] [1741980595.616800685] [sub2]: I heard: 'Hello, world! (1) (pub ptr: 0x55c9f936ddb0)' (sub ptr: 0x55c9f936ddb0)
[component_container-1] [INFO] [1741980596.616483840] [pub0]: Publishing: 'Hello, world! (2) (pub ptr: 0x55c9f936ddb0)'
[component_container-1] [INFO] [1741980596.616760486] [sub1]: I heard: 'Hello, world! (2) (pub ptr: 0x55c9f936ddb0)' (sub ptr: 0x55c9f936ddb0)
[component_container-1] [INFO] [1741980596.616810353] [sub2]: I heard: 'Hello, world! (2) (pub ptr: 0x55c9f936ddb0)' (sub ptr: 0x55c9f936ddb0)
[component_container-1] [INFO] [1741980597.616214495] [pub0]: Publishing: 'Hello, world! (3) (pub ptr: 0x55c9f936ddb0)'
[component_container-1] [INFO] [1741980597.616448397] [sub1]: I heard: 'Hello, world! (3) (pub ptr: 0x55c9f936ddb0)' (sub ptr: 0x55c9f936ddb0)
[component_container-1] [INFO] [1741980597.616487578] [sub2]: I heard: 'Hello, world! (3) (pub ptr: 0x55c9f936ddb0)' (sub ptr: 0x55c9f936ddb0)
```

## Contacts

If you have any questions or you want to know more about this work, please
contact one of the following contributors:

- Ricardo B. Sousa
  ([rbs@fe.up.pt](mailto:rbs@fe.up.pt),
  [github](https://github.com/sousarbarb/),
  [gitlab](https://gitlab.com/sousarbarb/),
  [gitlab](https://gitlab.inesctec.pt/ricardo.b.sousa),
  [orcid](https://orcid.org/0000-0003-4537-5095),
  [scholar](https://scholar.google.pt/citations?user=Bz2FMqYAAAAJ),
  [linkedin](https://www.linkedin.com/in/sousa-ricardob/),
  [youtube](https://www.youtube.com/channel/UCXTR8mMlG0VOC_06PKg5KBQ))
