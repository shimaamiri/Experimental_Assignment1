# Experimental Robotics Laboratory - Assignment 1

## Project Description

This assignment focuses on the utilization of a ROSbot to navigate an environment, locating and reaching four distinct Aruco markers. The task involves implementing this functionality initially in a simulated environment using Gazebo with a ROSbot equipped with a fixed camera. Subsequently, the solution is adapted for a real ROSbot in the laboratory setting. 
The robot starts at the coordinates (0, 0), and there are four markers in the environment with IDs 11, 12, 13, and 15. The markers have the following meanings:
- Marker 11: Rotate until you find Marker 12; then reach Marker 12.
- Marker 12: Rotate until you find Marker 13; then reach Marker 13.
- Marker 13: Rotate until you find Marker 15; then reach Marker 15.
- Marker 15: Done! (Task completion marker)
  
**Note:** "Reach Marker xxx" means that one side of Marker xxx must be at least 200 pixels in the camera frame.


## Required Packages
This project depends on few packages:

- **ArUco:**
You can download it here ([ArUco](https://github.com/CarmineD8/aruco_ros)) 
    
- **ROSbot Model:**
You can download it here ([ROSbot Model](https://github.com/husarion/rosbot_ros)) 
    
- **OpenCV:**
You can download it here ([OpenCV](https://github.com/ros-perception/vision_opencv))  


To download this project you need to enter following command in your root directory:
  
```bash
git clone https://github.com/shimaamiri/Experimental_Assignment1.git
```
Then you need to build the project in root directory:
```bash
catkin_make
```

## Architecture and Pseudocode

 This project consists of two nodes:
- **Robot_controller Node:** Controls robot behavior and its written in python.
- **Marker_publisher Node:** Detects Aruco markers and communicates information to the robot_controller node using a custum message called **Info**. This node is the modified version of Marker_publish.cpp from Aruco package.

You can find the pseudocode in the following:

```python

Initialize ROS node

Define a class for RobotController:
    Initialize variables for marker_list, marker_index, distance_th, misalignment_th, state, msg
    Initialize ROS publishers and subscribers for '/cmd_vel' and '/info' topics

    Callback function info_clbk(msg):
        Set self.msg to the received message

    Function search(marker_id):
        Create a Twist message for robot velocity
        If marker_id is equal to the current marker in marker_list:
            Log detection of marker
            Set angular velocity to 0
            Set state to 'align'
        Else:
            Log searching for marker
            Set angular velocity to 0.2
        Publish robot velocity

    Function align(marker_center):
        Log misalignment adjustment
        If the absolute difference between marker_center and 400 is greater than misalignment_th:
            Create a Twist message for robot velocity
            If marker_center is to the left of 400:
                Set angular velocity to 0.1 (turn left)
            Else:
                Set angular velocity to -0.1 (turn right)
            Publish robot velocity
        Else:
            Log alignment achieved
            Set state to 'drive'

    Function drive(marker_l):
        Log moving forward
        If marker_l is less than distance_th:
            Create a Twist message for robot velocity
            Set linear velocity to 0.2
            Publish robot velocity
        Else:
            Log reached the desired marker
            Increment marker_index
            If marker_index is less than the length of marker_list:
                Set state to 'search'
                Log moving to the next marker
            Else:
                Log all markers found
                Create a Twist message for robot velocity
                Set linear velocity to 0
                Publish robot velocity

    Main loop function main_loop():
        Set loop rate to 10 Hz
        While not rospy.is_shutdown():
            If state is 'search':
                Call search function with the current marker_id
            ElseIf state is 'align':
                Call align function with the current marker_center
            ElseIf state is 'drive':
                Call drive function with the current marker_l
            End If
            Sleep based on the loop rate

Main:
    Try:
        Create an instance of RobotController
        Call the main_loop function on the instance
    Except rospy.ROSInterruptException:
        Pass (do nothing)
```

## Simulation implementation

### How to Run 

```bash
roslaunch rosbot_bringup assignment.launch
```
### Video

Here is the simulation video: [Video Link](https://github.com/shimaamiri/exp1/assets/114082533/557a6603-cb2d-4cb0-8ea1-5774017435cc)

## Real robot Implementation 

### How to Run

In the assignment package go to the main branch then follow one of the following methods to run the code with real robot.

1. Share the ROS master with the ROSbot and execute the code from your PC. Ensure both are on the same network, and set the ROS_MASTER IP to match the ROSbot's.
2. Install the package directly on the ROSbot, considering it has ROS Melodic and OpenCV installed.

Finally run the dedicated launch file:

```bash
roslaunch rosbot_bringup assignment.launch
```

### Video

Here is the real robot video: [Video Link](https://github.com/shimaamiri/exp1/assets/114082533/557a6603-cb2d-4cb0-8ea1-5774017435cc)


## Drawback and Possible improvements

## Authors

-Shima Amiri Fard       5962794
-Mohammadreza Koolani   5
-Mohammad Saboori       5
-Reza Taleshi           5
