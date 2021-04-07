#!/usr/bin/env python3

import rospy
from tbd_ros_msgs.msg import (
    pollySpeechAction,
    pollySpeechGoal,
    pollySpeechResult
)
from std_msgs.msg import Bool
import boto3
from botocore.exceptions import BotoCoreError, ClientError
import actionlib
from contextlib import closing
import struct
import rospkg
import os
from tbd_polly_speech.HashTable import Item, HashTable
import wave


class PollyAudioLibrary(object):
    """ Interface to access the local stored copies of the synthesized audio
    """

    def __init__(self):

        # check if the user defined where the audio should be stored accoring to ROSParam, if not, default to package root
        rospack = rospkg.RosPack()
        rospy.loginfo('polly node\' namespace:{}'.format(
            rospy.get_namespace()))
        self._lib_directory = rospy.get_param(
            "polly_node/polly_audio_storage_path", os.path.join(rospack.get_path('tbd_polly_speech'), 'audio_storage'))
        rospy.loginfo('polly_speech will store audio at:{}'.format(
            self._lib_directory))
        # make directory if not exist
        if not os.path.exists(self._lib_directory):
            os.makedirs(self._lib_directory)

        self._h = HashTable(self._lib_directory)

    def save_text(self, text, voice_id, data):
        key = Item(voice_id, text)
        file_name = str(self._h.hashing(key)) + ".wav"

        # only save if unique
        if self._h.find(key) == None:
            self._h.insert(key)
            wav_path = os.path.join(self._lib_directory, file_name)
            # save the wave file
            wav_file = wave.open(wav_path, 'w')
            wav_file.setnchannels(1)
            wav_file.setsampwidth(2)
            wav_file.setframerate(16000)
            # write the frames
            wav_file.writeframes(data)
            wav_file.close()

            return wav_path
        return None

    def find_text(self, text, voice_id):
        """Returns the wav file if exist, None if not
        """
        key = Item(voice_id, text)
        file_name = str(self._h.hashing(key)) + ".wav"
        if self._h.find(key) != None:
            return os.path.join(self._lib_directory, file_name)
        else:
            rospy.logdebug("audiofile doesn't exist")
            return None


class PollyNode(object):

    def __init__(self):

        # Get ROS Params
        # debug flag
        # whether to skip the generation, useful when debugging and don't want to spend money on amazon
        self._no_audio_flag = rospy.get_param('~no_audio', False)
        self._no_break_flag = rospy.get_param('~no_break_if_no_audio', False)
        self._play_type = rospy.get_param('~output', 'sound_play')
        self._speak_signal_rate = rospy.get_param("~speak_signal_hz", 20)

        self._polly = boto3.client('polly', region_name='us-east-1')
        # speak signal publisher
        self.is_speaking = False
        self.pub = rospy.Publisher("speak_signal", Bool, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(
            1/self._speak_signal_rate), self.timer_callback)
        # setup the action lib server
        self._speak_server = actionlib.SimpleActionServer(
            "speak", pollySpeechAction, self._speak_callback, auto_start=False)
        self._speak_server.register_preempt_callback(self._preempt_callback)
        if self._play_type == 'tbd_audio_common':
            # get the message and start action server
            from tbd_ros_msgs.msg import (
                playAudioAction,
                playAudioGoal
            )
            self._tbd_audio_client = actionlib.SimpleActionClient(
                "playAudio", playAudioAction)
            self._tbd_imported_playAudioGoal = playAudioGoal
            self._tbd_audio_client.wait_for_server()

        elif self._play_type == 'sound_play':
            # import the clients and initialize it
            from sound_play.libsoundplay import SoundClient
            self._sound_play_client = SoundClient(blocking=True)

        # start the library
        self._audio_lib = PollyAudioLibrary()

        # start action server
        self._speak_server.start()

        rospy.loginfo("PollyNode ready")

    def timer_callback(self, event):
        """Function that will continuosly publish status
        """
        self.pub.publish(self.is_speaking)

    def _synthesize_speech(self, text, voice_id):
        # Call Amazon Poly and try to generate the speech
        self.is_speaking = False
        try:
            if text.startswith('<speak>'):
                response = self._polly.synthesize_speech(
                    Text=text, OutputFormat='pcm', VoiceId=voice_id, TextType='ssml')
            else:
                response = self._polly.synthesize_speech(
                    Text=text, OutputFormat='pcm', VoiceId=voice_id)
        except(BotoCoreError, ClientError) as error:
            print(error)
            return None

        if("AudioStream" in response):
            data = None
            with closing(response["AudioStream"]) as stream:
                data = stream.read()
            return data
        else:
            print("ERROR")
            return None

    def _preempt_callback(self):
        self.is_speaking = False
        if self._play_type == 'tbd_audio_common':
            # check if audio is running
            if self._tbd_audio_client.get_state() == actionlib.SimpleGoalState.ACTIVE:
                self._tbd_audio_client.cancel_goal()
                self.is_speaking = False
        elif self._play_type == 'sound_play':
            self._sound_play_client.stopAll()
            self.is_speaking = False

    def _speak_callback(self, goal):
        self.is_speaking = False
        complete = True
        rospy.loginfo('POLLY_SPEAK:{}'.format(goal.text))
        if not self._no_audio_flag:
            complete = self.speak(goal)
            self.is_speaking = False
        elif not self._no_break_flag:
            self.is_speaking = False
            num_of_words = len(goal.text.split(' '))
            duration = num_of_words * 0.4  # estimate 150 words per minute
            rospy.sleep(duration)
        result = pollySpeechResult()
        result.complete = complete
        self.is_speaking = False
        # check if speak failed because it is being preempted
        if not complete and self._speak_server.is_preempt_requested():
            rospy.loginfo('set aborted')
            self._speak_server.set_aborted(result)
        else:
            self._speak_server.set_succeeded(result)

    def speak(self, goal):
        self.is_speaking = True
        # sanitize the incoming text
        goal.text = goal.text.strip()

        data = None

        # try finding the text in the local stored library
        wav_path = self._audio_lib.find_text(goal.text, goal.voice_id)

        # synthesize speech if doesn't exist
        if wav_path is None:
            rospy.loginfo("synthesizing speech with AWS")
            data = self._synthesize_speech(goal.text, goal.voice_id)
            if data is None:
                self.is_speaking = False
                # this means that for some reason it wasn't able to generate the speech
                # return failure
                return False
            # check if server is preempted
            if self._speak_server.is_preempt_requested():
                self.is_speaking = False
                return False

            # save the data into a wav file
            wav_path = self._audio_lib.save_text(
                goal.text, goal.voice_id, data)
            if wav_path is None:
                self.is_speaking = False
                # this probably means there is a conflict, which is nearly impossible
                raise RuntimeError

        # play the wave file
        rospy.logdebug("POLLYSPEAK:playing wave file")
        if self._play_type == 'tbd_audio_common':
            self.is_speaking = True
            # get the wave file
            waveFile = wave.open(wav_path)
            num_of_frames = waveFile.getnframes() * waveFile.getsampwidth()
            # generate goal
            goal = self._tbd_imported_playAudioGoal()
            goal.soundFile = waveFile.readframes(num_of_frames)
            goal.rate = int(waveFile.getframerate())
            goal.size = num_of_frames
            # send to the goal server
            self._tbd_audio_client.send_goal_and_wait(goal)
            self.is_speaking = False

        elif self._play_type == 'sound_play':
            self.is_speaking = True
            self._sound_play_client.playWave(wav_path)
            self.is_speaking = False
        else:
            self.is_speaking = True
            rospy.logwarn(
                "unknown sound playing module {} in polly_speech".format(self._play_type))
            self.is_speaking = False
        rospy.logdebug("POLLYSPEAK:playing wave file completed")
        self.is_speaking = False
        # check whether we were pre-empted
        if self._speak_server.is_preempt_requested():
            self.is_speaking = False
            return False
        self.is_speaking = False
        return True


if __name__ == '__main__':
    rospy.init_node("polly_node", log_level=rospy.DEBUG)
    pl = PollyNode()
    rospy.spin()
