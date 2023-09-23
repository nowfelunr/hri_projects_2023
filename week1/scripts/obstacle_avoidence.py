#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoidance:
    def __init__(self):
        rospy.init_node('obstacle_avoidance', anonymous=True)
        rospy.Subscriber('/base_scan', LaserScan, self.laser_callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.twist_msg = Twist()

        # Parameters for obstacle avoidance
        self.min_distance = 0.5  # Minimum distance to stop the robot (Goal 3)
        self.turn_threshold = 0.2  # Threshold for obstacle detection (Goal 4)

    def laser_callback(self, data):
        # Check if there's an obstacle in front of the robot
        front_distance = min(data.ranges[0:45] + data.ranges[-45:])
        if front_distance < self.min_distance:  # Obstacle in front (Goal 3)
            self.twist_msg.linear.x = 0.0
            self.twist_msg.angular.z = 0.0
        else:  # No obstacle in front (Goal 3)
            self.twist_msg.linear.x = 0.2  # Adjust forward speed as needed (Goal 3)
            
            # Calculate turn direction based on obstacle detection (Goal 4)
            left_distance = min(data.ranges[0:90])
            right_distance = min(data.ranges[-90:])
            
            if left_distance < right_distance:
                self.twist_msg.angular.z = 0.2  # Turn left (Goal 4)
            else:
                self.twist_msg.angular.z = -0.2  # Turn right (Goal 4)

        # Publish the Twist message to control the robot
        self.pub.publish(self.twist_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        obstacle_avoidance = ObstacleAvoidance()
        obstacle_avoidance.run()
    except rospy.ROSInterruptException:
        pass
