#!/usr/bin/env python

import unittest
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from tbd_ros_msgs.msg import(
    pollySpeechGoal,
    pollySpeechAction
)
from std_msgs.msg import Bool
from threading import Event

PKG = 'tbd_polly_speech'

class TestInterface(unittest.TestCase):

    def __init__(self, *args, **kwargs):
        unittest.TestCase.__init__(self,*args,**kwargs)
        rospy.init_node("test_interface_node")

    def test_actionlib_interface(self):
        # make sure we can subscribe to all the relavent action message and channels.
        _polly_client = actionlib.SimpleActionClient("speak", pollySpeechAction)
        self.assertTrue(_polly_client.wait_for_server())

    def _signal_cb(self, msg: Bool):
        self._initial_signal = msg.data
        self._signal_received.set()

    def test_speak_signal_interface(self):
        self._initial_signal = True
        self._signal_received = Event()
        rospy.Subscriber('speak_signal', Bool, self._signal_cb, queue_size=1)
        # wait five seconds for signals
        self.assertTrue(self._signal_received.wait(5))
        self.assertTrue(not self._initial_signal)

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_interface', TestInterface)
    
