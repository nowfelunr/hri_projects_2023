#!/usr/bin/env python3
import rospy
from people_msgs.msg import PositionMeasurementArray
from geometry_msgs.msg import Point
from math import sqrt

def calculate_slope(point1, point2):
    """Calculate the slope between two points."""
    if point2.x - point1.x == 0:  # Avoid division by zero
        return float('inf')  # Infinite slope
    else:
        return (point2.y - point1.y) / (point2.x - point1.x)


def check_if_line(people_positions, delta=0.1):
    
    n_people = len(people_positions)
    if n_people < 2:
        return False  
    
    reference_slope = calculate_slope(people_positions[0], people_positions[1])

    for i in range(n_people - 1):
        for j in range(i + 1, n_people):
            slope = calculate_slope(people_positions[i], people_positions[j])
            if abs(slope - reference_slope) > delta:
                return False

    return True

def get_distance(point1, point2):
    """Calculate the distance between two points."""
    dx = point1.x - point2.x
    dy = point1.y - point2.y
    return sqrt(dx**2 + dy**2)

def find_farthest_points(people_positions):
    """Find the two points that are farthest apart."""
    max_distance = 0
    point1, point2 = None, None
    n_people = len(people_positions)
    for i in range(n_people):
        for j in range(i + 1, n_people):
            distance = get_distance(people_positions[i], people_positions[j])
            if distance > max_distance:
                max_distance = distance
                point1, point2 = people_positions[i], people_positions[j]
    return point1, point2

def find_circle_center(point1, point2):
    """Find the center of the circle given two points."""
    center_x = (point1.x + point2.x) / 2
    center_y = (point1.y + point2.y) / 2
    return Point(center_x, center_y, 0)


def check_if_circle(people_positions, delta=0.1):
    if len(people_positions) < 3:
        return False  # Need at least 3 people to form a circle

    point1, point2 = find_farthest_points(people_positions)
    if point1 is None or point2 is None:
        return False  # Not enough points to determine a circle

    center = find_circle_center(point1, point2)
    radius = get_distance(center, point1)

    for point in people_positions:
        if abs(get_distance(center, point) - radius) > delta:
            return False

    return True

def get_people_positions(data):
    """Extracts positions from PositionMeasurementArray."""
    return [Point(person.pos.x, person.pos.y, person.pos.z) for person in data.people]

def update_people_names(data, group_type):
    """Updates names of people based on detected group."""
    for i, person in enumerate(data.people):
        person.name = f'{group_type}_{i}_{person.name}'
    return data

def people_tracker_measurements_callback(data):
    people_positions = get_people_positions(data)
    is_line = check_if_line(people_positions, delta=1.0)
    is_circle = check_if_circle(people_positions, delta=1.5)

    group_type = 'line' if is_line else 'circle' if is_circle else 'unknown'
    updated_data = update_people_names(data, group_type)

    detected_groups_pub.publish(updated_data)

def main():
    rospy.init_node('people_group_detector')

    global detected_groups_pub
    detected_groups_pub = rospy.Publisher('/robot_0/detected_groups', PositionMeasurementArray, queue_size=10)

    rospy.Subscriber('/people_tracker_measurements', PositionMeasurementArray, people_tracker_measurements_callback)

    rospy.spin()

if __name__ == '__main__':
    main()
