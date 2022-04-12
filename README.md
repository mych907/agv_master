# AGV Master

ROS package for a Jetson Nano-based UGV (Jetracer).

The package includes OccupancyGrid map, A* algorithm for path generation, a controller (Pure pursuit and Stanley), subscribers for Marvelmind indoor GPS systems, and publishers for communication with Jetracer.

# Prerequisites

The AGV Master package requires ROS, a PC as ground control station, a prebuilt Jetracer, Marvelmind Indoor GPS, and an IMU.

## 1. Jetracer

Follow this link follow the instructions. After setup, clone this repo in your Jetracer workspace.

(https://github.com/mych907/cytron_jetracer)



## 2. ROS Setup

Assuming you have the basic dependencies installed, run the following command in /workspace/src to clone this repo.

Make or build depending on your preferences.

```
git clone https://github.com/mych907/agv_master
cd ..
catkin_make
```


Next, connect the PC and Jetracer to the same WiFi.

For PC, add the lines below in bashrc to export the ROS_MASTER_URI. Replace the <YOUR_IP_ADDRESS> with the PC's actual IP.

```
export ROS_MASTER_URI=http://<YOUR_IP_ADDRESS>:11311
export ROS_HOSTNAME=<YOUR_IP_ADDRESS>
```

Similarly, add the lines below in bashrc for Jetracer.

```
export ROS_MASTER_URI=http://<YOUR_IP_ADDRESS>:11311
export ROS_IP=<JETRACER_IP_ADDRESS>
```

## 3. Marvelmind Indoor GPS (Or any other equivalent localization tool)

This package depends on an unltrasonic indoor GPS system developed by Marvelmind Robotics. Use of any other localization such as visual tracking, odometry or even SLAM can be added to replace this. In case you have the Marvelmind beacons, follow the instructions below.

The ultrasonic GPS system from Marvelmind provides a ROS package from link. (https://github.com/MarvelmindRobotics/marvelmind_nav-release)

Clone the repo for your ROS distro version and follow the instructions from this link. (https://marvelmind.com/pics/marvelmind_ROS.pdf)

You may have to setup the system with Dashboard according to the manual provided from Marvelmind Robotics, if you have not done already.

## 4. IMU for Yaw Measurements

# Running on ROS

Run the following command to run the node:

```
rosrun agv_master agv_node.py
```

To visualize, open the configuration in the /rviz folder.


