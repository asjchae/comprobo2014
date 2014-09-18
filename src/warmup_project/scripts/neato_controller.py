#!/usr/bin/env python
# Software License Agreement (BSD License)


import rospy
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan

class ObstacleAvoider():

    def __init__(self):
        rospy.init_node('obstacleavoider', anonymous = True)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('scan', LaserScan, self.scan_around)
        self.quadrant = 0
        self.front_distance = -1.0
        self.rear_distance = -1.0
        self.right_distance = -1.0
        self.left_distance = -1.0

    def scan_around(self, msg):
        # http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html

        check_obstacle = []
        check_right = []
        check_left = []
        check_front = []
        check_rear = []

        
        # Check obstacle.
        # for i in range(30):
        #     if msg.ranges[330+i] > 0:
        #         check_obstacle.append(msg.ranges[330+i])
        #     if msg.ranges[30-i] > 0:
        #         check_obstacle.append(msg.ranges[30-i])
        # if len(check_obstacle) > 30:
        #     print "There's an obstacle"
        #     self.quadrant = 789


        # Check left
        for i in range(90):
            if msg.ranges[45+i] > 0:
                check_left.append(msg.ranges[45+i])
        if len(check_left) > 45:
            print "There's a something on the left"
            self.left_distance = sum(check_left)/float(len(check_left))
            self.quadrant = 1


        # Check front
        for i in range(45):
            if msg.ranges[315+i] > 0:
                check_front.append(msg.ranges[315+i])
            if msg.ranges[45-i] > 0:
                check_front.append(msg.ranges[45-i])
        if len(check_front) > 45:
            print "There's a something in the front"
            self.front_distance = sum(check_front)/float(len(check_front))
            self.quadrant = 2


        # Check right
        for i in range(90):
            if msg.ranges[225+i] > 0:
                check_right.append(msg.ranges[225+i])
        if len(check_right) > 45:
            print "There's a something on the right"
            self.right_distance = sum(check_right)/float(len(check_right))
            self.quadrant = 

        # Check rear
        for i in range(90):
            if msg.ranges[135+i] > 0:
                check_rear.append(msg.ranges[135+i])
        if len(check_rear) > 45:
            print "There's a something in the back"
            self.rear_distance = sum(check_rear)/float(len(check_rear))
            self.quadrant = 4


    def run(self):
        velocity_msg = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
        r = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():

            # MVP: Robot is placed in front of wall, facing wall. Robot approaches wall, turns and follows it.
            

            if self.front_distance > 1.0:
                velocity_msg = Twist(Vector3(0.5,0.0,0.0), Vector3(0.0,0.0,0.0)) # Approach wall
            elif self.front_distance < 1.0:
                velocity_msg = Twist(Vector3(0.0,0.0,0.0), Vector3(0.0,0.0,0.0)) # Stop
                velocity_msg = Twist(Vector3(0.0,0.0,0.0), Vector3(0.0,0.0,0.1)) # Rotate a little bit
                while self.left_distance != 1.0 or self.right_distance != 1.0:
                    velocity_msg = Twist(Vector3(0.0,0.0,0.0), Vector3(0.0,0.0,0.1)) # Rotate a little bit
                if self.left_distance == 1.0 or self.right_distance == 1.0:
                    velocity_msg = Twist(Vector3(0.5,0.0,0.0), Vector3(0.0,0.0,0.0)) # Go straight


            self.pub.publish(velocity_msg)
            
            r.sleep()

if __name__ == '__main__':
    try:
        node = ObstacleAvoider()
        node.run()
    except rospy.ROSInterruptException: pass

# Viewing the Laser Scan Data:
# Startup rviz by executing rosrun rviz rviz
# Add a display for the scan topic by clicking "add" on the display panel of rviz and clicking on the "by topic" tab and then selecting the "scan" topic.
# Make sure that the global options fixed frame is set to "odom"

# Create a map of your environment:
# Startup hector mapping by running roslaunch neato_2dnav hector_mapping_neato.launch
# Startup rviz by executing rosrun rviz rviz
# Add a display for the scan topic by clicking "add" on the display panel of rviz and clicking on the "by topic" tab and then selecting the "scan" topic.
# Add a display for the map topic by clicking "add" on the display panel of rviz and clicking on the "by topic" tab and then selecting the "map" topic.
# Make sure that the global options fixed frame is set to "map"