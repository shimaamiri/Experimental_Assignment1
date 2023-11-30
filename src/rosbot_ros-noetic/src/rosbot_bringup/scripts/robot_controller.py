#!/usr/bin/env python3

#required libraries
from syslog import LOG_INFO
import rospy
from aruco_ros.msg import info
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
import math


class RobotController:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('robot_controller')

        #Initial values
        self.marker_list = [11, 12, 13, 15] 
        self.marker_index = 0
        self.distance_th = 130
        self.misalignment_th = 0.5
        self.state = 'search'
        self.msg = info()
        
        #Publisher to publish velocity values on the topic /cmd_vel
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        #Subscriber to subscribe to custum message data from info.msg
        self.info_sub = rospy.Subscriber('/info', info, self.info_clbk)
       
    #Callback function for info_sub subscriber
    def info_clbk(self, msg):
        self.msg = msg
       
    #Function to search for markers
    def search(self, marker_id):
        robot_vel = Twist()
        if marker_id == self.marker_list[self.marker_index]:
            rospy.loginfo('detected marker %d' % marker_id)
            robot_vel.angular.z = 0
            self.state = 'align'
        else:    
            rospy.loginfo('Searching for marker %d' % marker_id)
            robot_vel.angular.z = 0.2
        self.cmd_vel_pub.publish(robot_vel)

    #Function to adjust marker center with screen center
    def align(self, marker_center):
        rospy.loginfo('Misaligned, adjusting...')
        if abs(marker_center - 400) > self.misalignment_th:
            robot_vel = Twist()
            if marker_center - 400 < 0:
                robot_vel.angular.z = 0.1
            else:
                robot_vel.angular.z = -0.1
            self.cmd_vel_pub.publish(robot_vel)
        else:
            rospy.loginfo('Aligned...')
            self.state = 'drive'

    #Function to go stright toward markers
    def drive(self, marker_l):
        rospy.loginfo('Moving forward...')
        if marker_l < self.distance_th:
            robot_vel = Twist()
            robot_vel.linear.x = 0.2
            self.cmd_vel_pub.publish(robot_vel)
        else:
            rospy.loginfo('Reached the desired marker!')
            self.marker_index += 1
            if self.marker_index < len(self.marker_list):
                self.state = 'search'
                rospy.loginfo('Moving to the next marker %d' % self.marker_list[self.marker_index])
            else:
                rospy.loginfo('All markers found!')
                robot_vel = Twist()
                robot_vel.linear.x = 0
                self.cmd_vel_pub.publish(robot_vel)


    #Main function to control robot behavior
    def main_loop(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
             if self.state == 'search':
                self.search(self.msg.marker_id)
             elif self.state == 'align':
                  self.align(self.msg.marker_center)
             elif self.state == 'drive':
                self.drive(self.msg.marker_l)
        rate.sleep()

if __name__ == '__main__':
    try:
        controller = RobotController()
        controller.main_loop()
    except rospy.ROSInterruptException:
        pass