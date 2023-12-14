#!/usr/bin/env python3
import rospy

from sensor_msgs.msg import JointState, string
from math import atan2, asin, sqrt, pi

from std_msgs.msg import String, Bool

key_poses = [
    {"HeadYaw": 0, "HeadPitch": 0},
    {"HeadYaw": pi/4, "HeadPitch": pi/4,}
 
]

def move():
   
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

    for i in range(len(key_poses) - 1):
        start_pose = key_poses[i]
        end_pose = key_poses[i + 1]
        for t in range(10):  
            fraction = t / 10.0
            interpolated_pose = interpolate_poses(start_pose, end_pose, fraction)

            joint_state_msg = JointState()
            joint_state_msg.header.stamp = rospy.Time.now()
            for joint, position in interpolated_pose.items():
                joint_state_msg.name.append(joint)
                joint_state_msg.position.append(position)

            pub.publish(joint_state_msg)


def interpolate_poses(pose1, pose2, fraction):

    interpolated_pose = {}
    for joint in pose1:
        interpolated_pose[joint] = pose1[joint] + fraction * (pose2[joint] - pose1[joint])
    return interpolated_pose



def speech_callback(data):
    pub = rospy.Publisher('/tts/phrase', String, queue_size=10)
    rospy.loginfo("Received speech: %s", data.data)
    text_received = data.data
    if text_received == "move":
        move()

def listen():
    rospy.init_node('listen_command', anonymous=True)

    rospy.Subscriber("/speech_recognition/final_result", String, speech_callback)

    tts_status_pub = rospy.Publisher('/tts/status', Bool, queue_size=10)
    tts_status_pub.publish(False)

    rospy.spin()
if __name__ == '__main__':
    try:
        listen()
    except rospy.ROSInterruptException:
        pass
