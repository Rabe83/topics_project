#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def laserscan_callback(msg: LaserScan):
    # Read the range values representing the right side (90 degrees) and left side (-90 degrees)
    range_right = msg.ranges[0]
    range_left = msg.ranges[719]

    # Read the range value representing the forward direction (0 degrees)
    range_forward = msg.ranges[360]

    print("Right Range:", range_right)
    print("Forward Range:", range_forward)
    print("Left Range:", range_left)

    cmd = Twist()

    if range_forward < 0.5:
        # Turn fast to the left
        cmd.linear.x = 0.1
        cmd.angular.z = 1.0
    elif range_right > 0.3:
        # Approach the wall
        cmd.linear.x = 0.1
        cmd.angular.z = -0.1
    elif range_right < 0.2:
        # Move away from the wall
        cmd.linear.x = 0.1
        cmd.angular.z = 0.1
    else:
        # Keep moving forward
        cmd.linear.x = 0.1
        cmd.angular.z = 0.0

    pub.publish(cmd)

if _name_ == '_main_':
    rospy.init_node("topics_project_node")
    rospy.loginfo("The node is subscribing to the LaserScan of the robot and publishing the cmd_vel to the robot in a control loop.....")
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    sub = rospy.Subscriber("/scan", LaserScan, callback=laserscan_callback)
    rospy.spin()