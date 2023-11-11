#!/usr/bin/env python3
import rospy
import tf2_ros
import geometry_msgs.msg


def publish_new_frame():
    rospy.init_node('publish_new_frame')
    br = tf2_ros.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        t = geometry_msgs.msg.TransformStamped()

        t.header.frame_id = "LFinger22_link"
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = "pointing_frame"
        t.transform.translation.x = 1.0  
        t.transform.rotation.w = 1.0  

        br.sendTransform(t)
        rate.sleep()

if __name__ == '__main__':
    publish_new_frame()