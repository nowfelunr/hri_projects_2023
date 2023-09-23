#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

def callback(scan_data):
    closest = scan_data.ranges[0]
  
    for i in range(1, len(scan_data.ranges)):
        if closest > scan_data.ranges[i]:
            closest = scan_data.ranges[i]
    
    rospy.loginfo(closest)
    

def listener():

   
    rospy.init_node('base_scan_listener')

    rospy.Subscriber('/base_scan', LaserScan, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
