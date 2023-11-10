#!/usr/bin/env python3
import rospy
from people_msgs.msg import People
from geometry_msgs.msg import Twist
from math import sqrt, atan2

def get_group_center(people):
    
    x_sum = sum(person.position.x for person in people)
    y_sum = sum(person.position.y for person in people)
    n = len(people)
    return x_sum / n, y_sum / n

def get_distance(x1, y1, x2, y2):
   
    return sqrt((x1 - x2)**2 + (y1 - y2)**2)

def get_velocity_command(current_x, current_y, target_x, target_y):
   
    cmd_vel = Twist()
    distance = get_distance(current_x, current_y, target_x, target_y)

    if distance < 0.1:  
        return cmd_vel

    angle_to_target = atan2(target_y - current_y, target_x - current_x)
    cmd_vel.linear.x = min(distance, 0.5)  
    cmd_vel.angular.z = angle_to_target  

    return cmd_vel

def people_callback(data):
    if not data.people:
        return

   
    robot_x, robot_y = 0.0, 0.0

   
    target_x, target_y = get_group_center(data.people)

  
    cmd_vel_msg = get_velocity_command(robot_x, robot_y, target_x, target_y)
    cmd_vel_pub.publish(cmd_vel_msg)

def main():
    rospy.init_node('move_to_group')

    global cmd_vel_pub
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    rospy.Subscriber('/detected_groups', People, people_callback)

    rospy.spin()

if __name__ == '__main__':
    main()
