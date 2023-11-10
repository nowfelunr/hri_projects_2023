#!/usr/bin/env python3
import rospy
from people_msgs.msg import PositionMeasurementArray
from math import sqrt


GROUP_THRESHOLD = 1.0  # meters, for example

def get_distance(person1, person2):
    dx = person1.pos.x - person2.pos.x
    dy = person1.pos.y - person2.pos.y
    return sqrt(dx**2 + dy**2)

def people_tracker_measurements_callback(data):
    detected_people = data.people
    n_people = len(detected_people)

    # Check each pair of detected people to see if they are close enough to be in a group
    for i in range(n_people):
        for j in range(i + 1, n_people):
            if get_distance(detected_people[i], detected_people[j]) < GROUP_THRESHOLD:
                rospy.loginfo(f"People {detected_people[i].name} and {detected_people[j].name} are in a group.")

def main():
    rospy.init_node('people_group_detector')

    rospy.Subscriber('/people_tracker_measurements', PositionMeasurementArray, people_tracker_measurements_callback)

    rospy.spin()

if __name__ == '__main__':
    main()
