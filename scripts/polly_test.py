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
    speak(polly_client, "<speak>This is a XML sentence</speak>", "Matthew")
    polly_client.wait_for_result()
    speak(polly_client, "I will be interrupted. Hello", "Emma")
    rospy.sleep(1)
    polly_client.cancel_goal()
    polly_client.wait_for_result()
    print(polly_client.get_result())


if __name__ == '__main__':
    rospy.init_node("polly_node_test")
    main()
