# amr-quadrotor
A repository developed base on RotroS. The original repository is extended for AMR simulation.
## How to install it
- You need a **Ubuntu 16.04** and **ROS Kinetic (desktop full)** (http://wiki.ros.org/kinetic/Installation/Ubuntu)
- Follow the following instruction to install **RotorS**(https://github.com/ethz-asl/rotors_simulator)
```
$ pip install future
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
$ wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
$ sudo apt-get update
$ sudo apt-get install ros-kinetic-desktop-full ros-kinetic-joy ros-kinetic-octomap-ros ros-kinetic-mavlink python-wstool python-catkin-tools protobuf-compiler libgoogle-glog-dev ros-kinetic-control-toolbox libgeographic-dev geographiclib-tools
$ sudo rosdep init
$ rosdep update
$ source /opt/ros/kinetic/setup.bash
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
$ wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install python-catkin-tools
$ mkdir [your_workspace]/src
$ cd [your_workspace]/src
$ catkin_init_workspace  # initialize your catkin workspace
$ wstool init
$ wget https://raw.githubusercontent.com/ethz-asl/rotors_simulator/master/rotors_hil.rosinstall
$ wstool merge rotors_hil.rosinstall
$ wstool update
```
Since we do not need the rotors_simulator/rotors_hil_interface, and it always throw out error while compiling, please do
```
$ cd [your_workspace]/src/rotors_simulator/rotors_hil_interface
$ touch CATKIN_IGNORE
```
After all above is done, clone this repository to the workspace
```
$ cd [your_workspace]/src
$ git clone https://github.com/0Jiahao/amr_quadrotor.git
```
Finally, we can complie them all together
```
$ cd [your_workspace]
$ catkin build
```
After installation the file the file tree should look like this
```
[your_workspace]/src
├── amr_quadrotor
├── CMakeLists.txt -> /opt/ros/kinetic/share/catkin/cmake/toplevel.cmake
├── mav_comm
├── mavlink
├── mavros
├── rotors_hil.rosinstall
└── rotors_simulator
```
## Examples
- **Manual control**
Connect your xbox joystick with the computer, run
```
$ roslaunch [your_workspace]/src/amr_quadrotor/launch/amr_quadrotor_manual_control.launch
```
You can now fly the quadrotor with your joystick.
- **Setpoint control**
Connect your xbox joystick with the computer, run
```
$ roslaunch [your_workspace]/src/amr_quadrotor/launch/amr_quadrotor_setpoint_control.launch
```
You can now control the setpoint position (green) with your joystick. 
## Reference
```
@Inbook{Furrer2016,
author="Furrer, Fadri
and Burri, Michael
and Achtelik, Markus
and Siegwart, Roland",
editor="Koubaa, Anis",
chapter="RotorS---A Modular Gazebo MAV Simulator Framework",
title="Robot Operating System (ROS): The Complete Reference (Volume 1)",
year="2016",
publisher="Springer International Publishing",
address="Cham",
pages="595--625",
isbn="978-3-319-26054-9",
doi="10.1007/978-3-319-26054-9_23",
url="http://dx.doi.org/10.1007/978-3-319-26054-9_23"
}
```
