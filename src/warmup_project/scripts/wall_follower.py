#!/usr/bin/env python
# Software License Agreement (BSD License


import rospy
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan

class WallFollower():

    def __init__(self):
        rospy.init_node('wallfollower', anonymous = True)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('scan', LaserScan, self.scan_around)
        self.front_distance = -1.0


    def scan_around(self, msg):
        # http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
        check_front = []


        # Check front
        for i in range(45):
            if msg.ranges[314+i] > 0:
                check_front.append(msg.ranges[314+i])
            if msg.ranges[45-i] > 0:
                check_front.append(msg.ranges[45-i])
        if len(check_front) > 45:
            self.front_distance = sum(check_front)/float(len(check_front))



    def run(self):
        
        r = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
        	if self.front_distance > .8:
        		follow_msg = Twist(Vector3(0.1, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
        	else:
        		follow_msg = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.1))

            	self.pub.publish(follow_msg)
            	r.sleep()

if __name__ == '__main__':
    try:
        node = WallFollower()
        node.run()
    except rospy.ROSInterruptException: pass
