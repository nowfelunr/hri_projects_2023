#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Bool

class QuestionResponseNode:
    def __init__(self):
        rospy.init_node('question_response_node', anonymous=True)
        self.question_pub = rospy.Publisher('/tts/phrase', String, queue_size=10)
        self.tts_status_pub = rospy.Publisher('/tts/status', Bool, queue_size=10)
        rospy.Subscriber("/speech_recognition/final_result", String, self.speech_callback)

    def ask_question(self, question):
        rospy.loginfo("Asking question: %s", question)
        pub = rospy.Publisher('/tts/phrase', String, queue_size=10)
      
        self.question_pub.publish(question)
        self.tts_status_pub.publish(False) 

    def speech_callback(self, data):
        rospy.loginfo("Received speech: %s", data.data)
        response = data.data.lower()

        if "yes" in response:
            self.handle_yes_response()
        elif "no" in response:
            self.handle_no_response()
        else:
            self.handle_unknown_response()

    def handle_yes_response(self):
        rospy.loginfo("User love CS")
    

    def handle_no_response(self):
        rospy.loginfo("User don't love CS :()")


    def handle_unknown_response(self):
        rospy.loginfo("I don't have any clue!")


    def run(self):
        # Ask the first question when the node starts
        pub = rospy.Publisher('/tts/phrase', String, queue_size=10)
        pub.publish("Hello")
        self.ask_question("Do you love computer science?")
        rospy.spin()

if __name__ == '__main__':
    question_response_node = QuestionResponseNode()
    question_response_node.run()
