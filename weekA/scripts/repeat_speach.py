#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Bool

def speech_callback(data):
    pub = rospy.Publisher('/tts/phrase', String, queue_size=10)
    rospy.loginfo("Received speech: %s", data.data)
    text_received = data.data
    pub.publish(text_received)



def repeat_speech_node():
    rospy.init_node('repeat_speech_node', anonymous=True)
    rospy.Subscriber("/speech_recognition/final_result", String, speech_callback)

    tts_status_pub = rospy.Publisher('/tts/status', Bool, queue_size=10)
    tts_status_pub.publish(False)

    rospy.spin()

if __name__ == '__main__':
    repeat_speech_node()