#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from math import pi


key_poses = [
    {'HeadYaw': 0.0, 'HeadPitch': 0.0, 'LHipYawPitch': 0.0, 'LHipRoll': 0.0, 'LHipPitch': 0.0, 'LKneePitch': 0.0, 'LAnklePitch': 0.0, 'LAnkleRoll': 0.0, 'RHipYawPitch': 0.0, 'RHipRoll': 0.0, 'RHipPitch': 0.0, 'RKneePitch': 0.0, 'RAnklePitch': 0.0, 'RAnkleRoll': 0.0, 'LShoulderPitch': 0.0, 'LShoulderRoll': 0.0, 'LElbowYaw': 0.0, 'LElbowRoll': 0.0, 'LWristYaw': 0.0, 'LHand': 0.0, 'RShoulderPitch': 0.0, 'RShoulderRoll': 0.0, 'RElbowYaw': 0.0, 'RElbowRoll': 0.0, 'RWristYaw': 0.0, 'RHand': 0.0, 'RFinger23': 0.0, 'RFinger13': 0.0, 'RFinger12': 0.0, 'LFinger21': 0.0, 'LFinger13': 0.0, 'LFinger11': 0.0, 'RFinger22': 0.0, 'LFinger22': 0.0, 'RFinger21': 0.0, 'LFinger12': 0.0, 'RFinger11': 0.0, 'LFinger23': 0.0, 'LThumb1': 0.0, 'RThumb1': 0.0, 'RThumb2': 0.0, 'LThumb2': 0.0},
    {'HeadYaw': -1.731523234,
 'HeadPitch': -0.51173076,
 'LHipYawPitch': -0.5089509007999999,
 'LHipRoll': 0.11601553250000002,
 'LHipPitch': -0.5770577109999999,
 'LKneePitch': 1.1113149456100002,
 'LAnklePitch': 0.12888350819999994,
 'LAnkleRoll': -0.01494934069999998,
 'RHipYawPitch': -0.5089509007999999,
 'RHipRoll': -0.00417357049999989,
 'RHipPitch': -0.770965231,
 'RKneePitch': 0.29925841504,
 'RAnklePitch': -0.08308623519999991,
 'RAnkleRoll': -0.024020209500000056,
 'LShoulderPitch': -1.2714244319999999,
 'LShoulderRoll': 0.5870275237,
 'LElbowYaw': 2.00641454,
 'LElbowRoll': -0.06132658449999995,
 'LWristYaw': -1.501409784,
 'LHand': 0.9037,
 'RShoulderPitch': -0.05381028600000004,
 'RShoulderRoll': -1.2931456372999999,
 'RElbowYaw': -0.663660194,
 'RElbowRoll': 1.3453378312000002,
 'RWristYaw': 0.6292351500000002,
 'RHand': 0.5137,
 'RFinger23': 0.5136481163000001,
 'RFinger13': 0.5136481163000001,
 'RFinger12': 0.5136481163000001,
 'LFinger21': 0.9036087262999999,
 'LFinger13': 0.9036087262999999,
 'LFinger11': 0.9036087262999999,
 'RFinger22': 0.5136481163000001,
 'LFinger22': 0.9036087262999999,
 'RFinger21': 0.5136481163000001,
 'LFinger12': 0.9036087262999999,
 'RFinger11': 0.5136481163000001,
 'LFinger23': 0.9036087262999999,
 'LThumb1': 0.9036087262999999,
 'RThumb1': 0.5136481163000001,
 'RThumb2': 0.5136481163000001,
 'LThumb2': 0.9036087262999999}
 
]

def interpolate_poses(pose1, pose2, fraction):

    interpolated_pose = {}
    for joint in pose1:
        interpolated_pose[joint] = pose1[joint] + fraction * (pose2[joint] - pose1[joint])
    return interpolated_pose

def animate_nao():
    rospy.init_node('nao_animator')
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

    rate = rospy.Rate(10)  

    while not rospy.is_shutdown():
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
                rate.sleep()

if __name__ == '__main__':
    try:
        animate_nao()
    except rospy.ROSInterruptException:
        pass
