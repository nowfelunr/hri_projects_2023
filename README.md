# **HRI Course Assignments README**

This README provides instructions for running various robotics assignments using ROS. Each assignment focuses on different aspects of robotics, such as obstacle avoidance, gesture recognition, and speech generation.

## Assignment 1: Goal Moving/Obstacle Avoidance (Week 2)

**Steps to run:**
1. Launch the simulation environment:
`roslaunch week2 person_world.launch`
2. Run RViz for visualization:
`rosrun rviz rviz -d ~/hri2023/src/hri_projects_2023/week2/person.rviz`
3. Launch the leg detector:
`roslaunch week2 leg_detector.launch`
4. Broadcast the robot's position:
`rosrun week2 robot_pos_brdcst.py`
5. Broadcast the positions of people:
`rosrun week2 people_pos_brdcst.py`
6. Run the obstacle avoidance script:
`rosrun week2 obstacle_avoidence.py`

## Assignment 2: Look at Hand / Look at Where the Hand is Pointing (Week 8)

### Problem 1
**Steps to run:**
1. Launch the robot state publisher:
`roslaunch nao_description robot_state_publisher.launch`
2. Open RViz and load the robot model:
`rviz`
Then open the appropriate RViz file and select RobotModel.
3. Run the joint state publisher GUI (close after opening):
`rosrun joint_state_publisher_gui joint_state_publisher_gui`
4. Execute the script to make the robot look at its hand:
`rosrun week8 look_at_hand.py`

### Problem 2
**Steps to run:**
Follow the same first three steps as in Problem 1, then:
1. Run the frame broadcaster:
`rosrun week8 new_frame_broadcaster.py`
2. Execute the script to make the robot look at the new frame:
`rosrun week8 look_at_new_frame.py`

### Problem 3
**Steps to Run:**
Follow the same steps as in Problem 1, then:
1. Run the keyframe publisher:
`rosrun week7 keyframe_publisher.py`

## Assignment 3: Speech Generation (Week A)

### Problem 1
**Steps to run:**
1. Launch the speech recognition node:
`roslaunch ros_vosk ros_vosk.launch`
2. Run the script to repeat speech:
`rosrun weekA repeat_speach.py`

### Problem 2
**Steps to run:**
1. Launch the speech recognition node:
`roslaunch ros_vosk ros_vosk.launch`
2. Follow the same steps as in Assignment 2, Problem 1.
3. Execute the script to listen for a command and respond:
`rosrun weekA listen_command.py`
4. Then say "move" to initiate the command.

### Problem 3
**Steps to run:**
1. Launch the speech recognition node:
`roslaunch ros_vosk ros_vosk.launch`
2. Run the script to ask a question:
`rosrun weekA ask_question.py`
