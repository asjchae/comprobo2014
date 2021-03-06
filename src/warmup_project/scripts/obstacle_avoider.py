#!/usr/bin/env python
# Software License Agreement (BSD License

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
        self.sub = rospy.Subscriber('scan', LaserScan, self.scan_obstacles)
        self.front_distance = 1
        self.right_distance = 1


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

    def run(self):
        
        r = rospy.Rate(10) # 10hz
        obstacle_msg = Twist(Vector3(0.2, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))

        while not rospy.is_shutdown():
            if (self.front_distance < .8) and (self.front_distance > 0):
                obstacle_msg = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.2))
            else:
                obstacle_msg = Twist(Vector3(0.2, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
                if (self.right_distance > .3) and (self.right_distance < .8) and (self.front_distance > 1):
                    obstacle_msg = Twist(Vector3(0.05, 0.0, 0.0), Vector3(0.0, 0.0, -0.2))     

            self.pub.publish(obstacle_msg)
            r.sleep()


if __name__ == '__main__':
    try:
        node = ObstacleAvoider()
        node.run()
    except rospy.ROSInterruptException: pass
