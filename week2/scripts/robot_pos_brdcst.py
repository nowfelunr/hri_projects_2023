#!/usr/bin/env python3
import rospy

# Because of transformations
import tf_conversions

import tf2_ros
import geometry_msgs.msg

from nav_msgs.msg import Odometry

def broadcast_callback(pos_measurement):
    
        
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()

        t.header.stamp = rospy.Time

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = pos_measurement.header.frame_id
        t.child_frame_id = "robot_frame"
        t.transform.translation.x = pos_measurement.pose.pose.position.x
        t.transform.translation.y = pos_measurement.pose.pose.position.y
        t.transform.translation.z = pos_measurement.pose.pose.position.z
       
        t.transform.rotation.x = pos_measurement.pose.pose.orientation.x
        t.transform.rotation.y = pos_measurement.pose.pose.orientation.y
        t.transform.rotation.z = pos_measurement.pose.pose.orientation.z
        t.transform.rotation.w = pos_measurement.pose.pose.orientation.w
        br.sendTransform(t)



if __name__ == "__main__":
    rospy.init_node("robot_pos_broadcaster")

    rospy.Subscriber("/base_pose_ground_truth", Odometry, broadcast_callback)

    rospy.spin()