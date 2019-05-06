#!/usr/bin/env python

import rospy
from success_ros_msgs.msg import(
    pollySpeechGoal,
    pollySpeechAction
)
import actionlib

def speak(polly_client, text, voice_id="Joanna"):
    goal = pollySpeechGoal()
    goal.text = text
    goal.voice_id = voice_id
    polly_client.send_goal(goal)

def main():
    polly_client = actionlib.SimpleActionClient("success_polly_speech/speak", pollySpeechAction)
    polly_client.wait_for_server()
    rospy.loginfo('found server')
    speak(polly_client, "Lalalalalalalalalalalalalalalalala.", "Matthew")
    polly_client.wait_for_result()
    rospy.sleep(1)
    speak(polly_client, "This is a very long sentence that I will not be saying anymore. I will need a lot of data to store this sentence.", "Matthew")
    polly_client.wait_for_result()
    rospy.sleep(5)


if __name__ == '__main__':
    rospy.init_node("polly_node_test")
    main()
