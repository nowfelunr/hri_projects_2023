#!/usr/bin/env python3
import rospy

# Because of transformations
import tf_conversions

import tf2_ros
import geometry_msgs.msg

from people_msgs.msg import PositionMeasurementArray

def broadcast_callback(pos_measurement_arr):
    for person in pos_measurement_arr.people:
        if person.header.frame_id is not None and person.header.frame_id == 'odom' :
            br = tf2_ros.TransformBroadcaster()
            t = geometry_msgs.msg.TransformStamped()

            t.header.stamp = rospy.Time

            t.header.stamp = rospy.Time.now()
            t.header.frame_id = person.header.frame_id
            t.child_frame_id = "person_frame"
            t.transform.translation.x = person.pos.x
            t.transform.translation.y = person.pos.y
            t.transform.translation.z = 0.0
            q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            br.sendTransform(t)



if __name__ == "__main__":
    rospy.init_node("people_pos_broadcaster")

    rospy.Subscriber("/people_tracker_measurements", PositionMeasurementArray, broadcast_callback)

    rospy.spin()