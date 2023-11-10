#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from math import pi, sin
import time

# Define key poses for Nao robot. Replace with actual joint names and values.
key_poses = [
    {"HeadYaw": 0, "HeadPitch": 0, "LHipYawPitch": 0, "LKneePitch": 0},
    {"HeadYaw": pi/4, "HeadPitch": pi/4, "LHipYawPitch": -pi/4, "LKneePitch": pi/4}
 
]

def interpolate_poses(pose1, pose2, fraction):
    """Interpolate between two poses."""
    interpolated_pose = {}
    for joint in pose1:
        interpolated_pose[joint] = pose1[joint] + fraction * (pose2[joint] - pose1[joint])
    return interpolated_pose

def animate_nao():
    rospy.init_node('nao_animator')
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        for i in range(len(key_poses) - 1):
            start_pose = key_poses[i]
            end_pose = key_poses[i + 1]
            for t in range(10):  # Duration of each pose transition
                fraction = t / 10.0
                interpolated_pose = interpolate_poses(start_pose, end_pose, fraction)

                joint_state_msg = JointState()
                joint_state_msg.header.stamp = rospy.Time.now()
                for joint, position in interpolated_pose.items():
                    joint_state_msg.name.append(joint)
                    joint_state_msg.position.append(position)

                pub.publish(joint_state_msg)
                rate.sleep()

if __name__ == '__main__':
    try:
        animate_nao()
    except rospy.ROSInterruptException:
        pass
