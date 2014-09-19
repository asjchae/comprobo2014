#!/usr/bin/env python
# Software License Agreement (BSD License


import rospy
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan

class Controller():

    def __init__(self):
        rospy.init_node('controller', anonymous = True)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('scan', LaserScan, self.scan_wall)
        self.front_distance = 1
        self.right_distance = 1


    def scan_wall(self, msg):
        # http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
        check_front = []
        check_right = []

        # Check front
        for i in range(15):
            if msg.ranges[344+i] > 0:
                check_front.append(msg.ranges[344+i])
            if msg.ranges[15-i] > 0:
                check_front.append(msg.ranges[15-i])
        if len(check_front) > 0:
            self.front_distance = sum(check_front)/float(len(check_front))
        # else:
        #     self.front_distance = -1

        # Check right
        for i in range(30):
            if msg.ranges[255+i] > 0:
                check_right.append(msg.ranges[255+i])
        if len(check_right) > 0:
            self.right_distance = sum(check_right)/float(len(check_right))
        # else:
        #     self.right_distance = 1


    def wallfollow(self):
        
        r = rospy.Rate(10) # 10hz

        while not rospy.is_shutdown():
            if (self.front_distance < .8) and (self.front_distance > 0):
                follow_msg = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.1))
            else:
                follow_msg = Twist(Vector3(0.1, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
                if (self.right_distance > .5) and (self.right_distance < .8) and (self.front_distance > 1):
                    follow_msg = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, -0.1))                

            self.pub.publish(follow_msg)
            r.sleep()






    def scan_obstacles(self, msg):
        check_front = []
        check_right = []

        # Check front
        for i in range(45):
            if msg.ranges[314+i] > 0:
                check_front.append(msg.ranges[314+i])
            if msg.ranges[45-i] > 0:
                check_front.append(msg.ranges[45-i])
        if len(check_front) > 0:
            self.front_distance = sum(check_front)/float(len(check_front))       

        # Check right
        for i in range(30):
            if msg.ranges[255+i] > 0:
                check_right.append(msg.ranges[255+i])
        if len(check_right) > 0:
            self.right_distance = sum(check_right)/float(len(check_right))

    def obstacleavoid(self):
        
        r = rospy.Rate(10) # 10hz
        obstacle_msg = Twist(Vector3(0.1, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))

        while not rospy.is_shutdown():
            if (self.front_distance < .8) and (self.front_distance > 0):
                obstacle_msg = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.1))
            else:
                obstacle_msg = Twist(Vector3(0.2, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
                if (self.right_distance > .3) and (self.right_distance < .8) and (self.front_distance > 1):
                    obstacle_msg = Twist(Vector3(0.05, 0.0, 0.0), Vector3(0.0, 0.0, -0.2))     

            self.pub.publish(obstacle_msg)
            r.sleep()

if __name__ == '__main__':
    try:
        node = Controller()
        node.wallfollow()
        node.obstacleavoid()
    except rospy.ROSInterruptException: pass