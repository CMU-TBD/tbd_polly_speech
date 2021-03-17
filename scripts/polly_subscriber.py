#!/usr/bin/env python
from std_msgs.msg import Bool
import os
import rospy


def callback(message):
    print(message)
    return


if __name__ == "__main__":
    """This file runs a subscriber to the polly speaking topic and prints output
    """
    rospy.init_node("polly_speech_sub")
    rospy.Subscriber("polly_speaking", Bool, callback, queue_size=10)
    rospy.spin()
