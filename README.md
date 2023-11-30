# Experimental Robotics Laboratory - Assignment 1

## Introduction

This assignment focuses on the utilization of a ROSbot to navigate an environment, locating and reaching four distinct Aruco markers. The task involves implementing this functionality initially in a simulated environment using Gazebo with a ROSbot equipped with a fixed camera. Subsequently, the solution is adapted for a real ROSbot in the laboratory setting. 

## Required Packages

To run the project, download the following ROS packages in your work space:

- **ArUco:** Download the ArUco package for ROS Noetic.
```bash
git clone https://github.com/CarmineD8/aruco_ros
```

    If using a ROS Melodic-enabled ROSbot:
    ```bash
    git clone https://github.com/pal-robotics/aruco_ros
    git checkout melodic-devel
    ```
    
- **ROSbot Model:** Obtain the ROSbot model for simulation and branch on noetic.
```bash
git clone https://github.com/husarion/rosbot_ros
```
    
- **OpenCV:** Download the ROS-compatible version (Noetic) if not present. 
```bash
git clone https://github.com/ros-perception/vision_opencv
```


- **Assignment Package:** 
The package contains two branches:
- **main:** For the ROSbot implementation with a fixed camera.
- **simulation:** For simulation implementation with a fixed camera.

```bash
git clone https://github.com/giuliab00/exp1
```

## Architecture and Pseudocode

 This project consists of two nodes:
- **Robot_controller Node:** Controls robot behavior and its written in python.
- **Marker Node:** Detects Aruco markers and communicates information to the robot_controller node using a custum message called **Info**. This node is the modified version of Marker_publish.cpp from Aruco package.

## Real robot Implementation 

### How to Run

In the assignment package go to the main branch then follow one of the following methods to run the code with real robot.

1. Share the ROS master with the ROSbot and execute the code from your PC. Ensure both are on the same network, and set the ROS_MASTER IP to match the ROSbot's.
2. Install the package directly on the ROSbot, considering it has ROS Melodic and OpenCV installed.

Finally run the dedicated launch file for real robot:

```bash
roslaunch exp1 laboratorium.launch
```

### Video

View the ROSbot in action: [Video Link](https://github.com/shimaamiri/exp1/assets/114082533/557a6603-cb2d-4cb0-8ea1-5774017435cc)

## Simulation implementation

### How to Run 

If on the simulation branch, initiate the simulation with a rotating camera:
```bash
roslaunch exp1 run.launch
```
### Video

## Drawback and Possible improvements

## Authors
Shima Amiri Fard
