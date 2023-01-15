# Mobile Robotic Base


Repository for the description and gazebo simulation of a Mobile Robotic Base, a project developed for the completion of the WPI Major Qualifying Project requirement. Last updated: May 2022

![Robot Render](https://github.com/RoboticMobileBaseMQP/mobile-base/blob/main/docs/Robot%20Render.png?raw=true)

## Installation
There are two types of installation necessary to communicate with the mobile base. If only working with simulation, then only the Host Computer installation needs to be followed. However, if working with the real Mobile Base, the Raspberry Pi installation will also need to be followed.

### Host Computer Installation 

This project was developed using ROS as the framework of choice. Download and install ROS Noetic from [here](http://wiki.ros.org/noetic/Installation). Create a workspace using the following commands: 

```
mkdir MQP_ws/src
cd MQP_ws
catkin_make
```

Remove the generated CMakeLists.txt with

```
cd src
sudo rm CMakeLists.txt
```


Next, clone this repo using 

```
git clone https://github.com/RoboticMobileBaseMQP/mobile-base.git
```

Next install the repository for the kortex arm:

```
cd kortex_arm
git clone https://github.com/Kinovarobotics/ros_kortex.git -b noetic-devel
```

`ros_kortex` requires additional steps to build the drivers for the gen3 - follow them [here](https://github.com/Kinovarobotics/ros_kortex).

Our simulation also requires a little tweak to the gen3 URDF. Comment out lines 25-32 and 64-68 in `kortex_arm/ros_kortex/kortex_description/robots/kortex_robot.xacro` to unfix the robot from the world link in Gazebo.

Then install missing dependencies with 

```
cd ../..
rosdep install --from-paths src --ignore-src -r -y
```

Finally, set up your network for ROS. Add the following lines to ~/.bashrc:

```
export ROS_MASTER_URI=http://localhost:11311/
export ROS_HOSTNAME=`hostname -I | cut -f1 -d' '`
```

Then add `raspberrypi <rpi ip address>` to /etc/hosts.

### Raspberry Pi Installation

The Raspberry Pi requires a lot of setup for this project. First, install a 64bit debian “Buster” OS. We recommend downloading the image from [here](https://downloads.raspberrypi.org/raspios_arm64/images/raspios_arm64-2021-05-28/) and using the Raspberry Pi Imager to install it. RaspiOS should now boot up when turning on the Pi.

To control the Franka Emika Panda, a fully preemptible real-time OS is required. Follow the instructions listed [here](https://www.instructables.com/64bit-RT-Kernel-Compilation-for-Raspberry-Pi-4B-/). Note that you will need to use a different kernel version (we used rpi-5.10) and also a different patch version (we used 5.10.104) - pick the version closest to what’s already installed on the Pi. Make sure to follow instructions for `menuconfig` - additional details for enabling the real time preemption model can be found [here](https://unix.stackexchange.com/questions/582075/trouble-selecting-fully-preemptible-kernel-real-time-when-configuring-compil). 

Once the OS is patched, ROS needs to be built. Follow [these instructions](https://varhowto.com/install-ros-noetic-raspberry-pi-4/) for installing ROS Noetic on the Raspberry Pi. 

With ROS installed, the repositories can be installed. Follow the instructions for installing this repository and the kortex_arm repository above. There are 2 additional notes when building kortex_arm:

1. Build the kortex API with a CONAN_TARGET_PLATFORM of artik710. This makes it build properly

2. Delete any packages in kortex_arm that require Gazebo. Gazebo cannot be installed on a Raspberry Pi, and you won’t be running simulations on it, so the packages are unnecessary. 

Lastly, set up the ROS network. Add the following lines to ~/.bashrc:
```
export ROS_MASTER_URI=<host computer IP>:11311/
export ROS_HOSTNAME=`hostname -I | cut -f1 -d' '`
```

One final note: the `franka_description` repository only includes the URDF information for the simulation, as we didn’t have enough time to implement it on the Pi. To actually control the Panda arm, you’ll need to clone the full repository from [here](https://github.com/frankaemika/franka_ros) and monkey it with it on the Pi. Good luck!

## Usage

This simulation is still in development. The following is a description of some of the launch files included in this repository.

### simulated_arm_and_base.launch
`roslaunch mobile_base_simulation simulated_arm_and_base.launch arm:=xyz`

Simulates the mobile robot in Gazebo. Use `arm:=` to specify which arm should be loaded. Available arms are `panda` and `gen3`.

### base_sim_alone.launch
`roslaunch mobile_base_simulation base_sim_alone.launch`

Simulates only the mobile base. Helpful for debugging teleop modes.

### base_teleop.launch
`roslaunch base_package base_teleop.launch`

Launches the nodes necessary to connect to the mobile base through the Raspberry Pi. Control the mobile base with an xbox 360 controller. 

### remote_nodes.launch
`roslaunch base_package remote_nodes.launch`

Launch this file on the Raspberry Pi for controlling the mobile base and interfacing with the robot arms.

### host_moveit.launch
`roslaunch kortex_insert_description host_moveit.launch`

Launches MoveIt! on the host computer. Used to interface with robot arms when connected to the Pi.

### remote_kinova_driver.launch
`roslaunch kortex_insert_description remote_kinova_driver.launch`

Launches the drivers necessary for the Kinova Gen3 without moveit launching. This should be launched on the Pi.

###  actual_arm_and_base.launch
`roslaunch mobile_base_description actual_arm_and_base.launch`

Launches the drivers for the arm and base together. Run on the host computer.

###  state_machine.launch
`roslaunch task_manager state_machine.launch`

Launches the arm drivers and autonomous state machine together. This is untested and should be proceeded with caution.

![State Machine Diagram](https://github.com/RoboticMobileBaseMQP/mobile-base/blob/main/docs/Mobile%20Base%20State%20Diagram.png?raw=true)
