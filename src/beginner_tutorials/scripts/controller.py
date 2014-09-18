#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

def getch():
    """ Return the next character typed on the keyboard """
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


# Proportional Control
# Velocity = -c ( error )



def scan_received(msg, pub): # msg type is LaserScan
    """ Processes data from the laser scanner, msg is of type sensor_msgs/LaserScan """
    global velocity
    valid_ranges = []
    for i in range(5):
        if msgs.ranges[i] > 0 and msg.ranges[i] < 8:
            valid_ranges.append(msg.ranges[i])
    if len(valid_ranges) > 0
        mean_distance = sum(valid_ranges)/float(len(valid_ranges))
        # message to execute proportional control
        velocity = .2 * (mean_distance - 1.0) # a constant times the error
        # msg = Twist(Vector3(velocity, 0.0, 0.0), Vector 3(0.0, 0.0, 0.0))
        # pub.publish(msg)
    else:
        mean_distance = -1.0
        velocity = .2 * mean_distance - 1.0)


def teleop():
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10) # talking to cmd_vel, of type Twist learn about it by doing "rostopic type /cmd_vel"
    sub = rospy.Subscriber('scan', LaserScan, scan_received, pub) # callback processes data
    rospy.init_node('teleop', anonymous=True)
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        msg = Twist(Vector3(velocity, 0.0, 0.0), Vector 3(0.0, 0.0, 0.0))
        # ch = getch()
        # if ch == 'i':
        #     msg = Twist(Vector3(1.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
        # elif ch == 'k':
        #     msg = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
        # elif ch == 'j':
        #     msg = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 1.0))
        # elif ch == 'q':
        #     break
        pub.publish(msg) # Give it to the Publisher
        r.sleep()

if __name__ == '__main__':
    try:
        teleop()
    except rospy.ROSInterruptException: pass

# rosrun rviz
# rospy.Subscriber("chatter, String", callback) - setting up a subscriber init_node
# rostsopic list
# rosmsg show sensor_msgs/LaserScan
# rostopic echo scan