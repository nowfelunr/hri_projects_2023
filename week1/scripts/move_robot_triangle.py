#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import math

def square_move():
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('square_mover', anonymous=True)
    rate = rospy.Rate(10)  # 1 Hz

    # Initialize a Twist message
    twist = Twist()

    # Set linear and angular velocities for the triangle
    linear_speed = 0.2  # Adjust as needed
    angular_speed = math.pi / 6  # 30 degrees per second (adjust as needed)

    while not rospy.is_shutdown():
        # Move forward
        twist.linear.x = linear_speed
        twist.angular.z = 0
        for _ in range(10):  # Move forward for 10 seconds (adjust as needed)
            pub.publish(twist)
            rate.sleep()

        # Stop and turn 120 degrees (clockwise)
        twist.linear.x = 0
        twist.angular.z = angular_speed
        for _ in range(4):  # Turn for 4 seconds to make a 120-degree turn (adjust as needed)
            pub.publish(twist)
            rate.sleep()

    # Stop the robot when done
    twist.linear.x = 0
    twist.angular.z = 0
    pub.publish(twist)

if __name__ == '__main__':
    try:
        square_move()
    except rospy.ROSInterruptException:
        pass
