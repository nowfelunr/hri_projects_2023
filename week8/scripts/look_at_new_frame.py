#!/usr/bin/env python3
import rospy
import tf2_ros
from sensor_msgs.msg import JointState
from math import atan2, asin, sqrt

def interpolate_angles(start, end, fraction):
    return start + fraction * (end - start)


def calculate_head_angles(target, head):

    dx = target.x - head.x
    dy = target.y - head.y
    dz = target.z - head.z

    distance = sqrt(dx**2 + dy**2 + dz**2)
    yaw = atan2(dy, dx)
    pitch = -asin(dz / distance)  
    return yaw, pitch
    

def head_look_at_new_frame():
    rospy.init_node('head_look_at_new_frame')
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

    rate = rospy.Rate(10)
    
    current_yaw, current_pitch = 0.0, 0.0
    while not rospy.is_shutdown():
        try:

            trans_point = tf_buffer.lookup_transform('base_link', 'pointing_frame', rospy.Time())
            trans_head = tf_buffer.lookup_transform('base_link', 'Head', rospy.Time())

            target_yaw, target_pitch = calculate_head_angles(trans_point.transform.translation, trans_head.transform.translation)

            for i in range(0, 11): 
                interp_yaw = interpolate_angles(current_yaw, target_yaw, i / 10.0)
                interp_pitch = interpolate_angles(current_pitch, target_pitch, i / 10.0)

                joint_msg = JointState()
                joint_msg.header.stamp = rospy.Time.now()
                joint_msg.name = ['HeadYaw', 'HeadPitch']
                joint_msg.position = [interp_yaw, interp_pitch]
                pub.publish(joint_msg)
                rate.sleep()
            
            current_yaw, current_pitch = target_yaw, target_pitch

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue

        rate.sleep()

if __name__ == '__main__':
    head_look_at_new_frame()
