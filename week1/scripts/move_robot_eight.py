#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist


def square_move():
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('eight_mover', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    twist = Twist()

    # Set linear and angular velocities for moving in a square
    linear_speed = 0.2  # Adjust as needed
    angular_speed = 0.2  # Adjust as needed

    while not rospy.is_shutdown():
        # Move forward
        twist.linear.x = linear_speed
        twist.angular.z = 0
        for _ in range(5):
            pub.publish(twist)
            rate.sleep()

        # Turn 90 degrees (clockwise)
        twist.linear.x = 0
        twist.angular.z = angular_speed
        for _ in range(4):
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
