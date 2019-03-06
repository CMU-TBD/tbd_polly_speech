#!/usr/bin/env python

import rospy
from success_ros_msgs.msg import (
    pollySpeechAction,
    pollySpeechGoal,
    pollySpeechResult
)

import boto3
from botocore.exceptions import BotoCoreError, ClientError
import actionlib
from contextlib import closing
import struct
import rospkg
import os
from masterfile import Item, HashTable
import wave
from sound_play.libsoundplay import SoundClient


class PollyAudioLibrary(object):
    """ Interface to access the local stored copies of the synthesized audio
    """
    def __init__(self):

        #check if the user defined where the audio should be stored accoring to ROSParam, if not, default to package root
        rospack = rospkg.RosPack()
        rospy.loginfo('polly node\' namespace:{}'.format(rospy.get_namespace()))
        self._lib_directory = rospy.get_param("polly_node/polly_audio_storage_path", os.path.join(rospack.get_path('success_polly_speech'), 'audio_storage'))
        rospy.loginfo('polly_speech will store audio at:{}'.format(self._lib_directory))
        #make directory if not exist
        if not os.path.exists(self._lib_directory):
            os.makedirs(self._lib_directory)

        self._h = HashTable(10 ** 32, self._lib_directory)

    def save_text(self, text, voice_id, data):
        key = Item(voice_id, text)
        file_name = str(self._h.hashing(key)) + ".wav"

        # only save if unique
        if self._h.find(key) == None:
            self._h.insert(key)
            wav_path = os.path.join(self._lib_directory, file_name)
            #save the wave file
            wav_file = wave.open(wav_path,'w')
            wav_file.setnchannels(1)
            wav_file.setsampwidth(2)
            wav_file.setframerate(16000)
            #write the frames
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
        ## Get ROS Params
        # debug flag
        self._no_audio_flag = rospy.get_param('no_audio', False) #whether to skip the generation, useful when debugging and don't want to spend money on amazon

        self._polly = boto3.client('polly', region_name='us-east-1')
        #setup the action lib server
        self._speak_server = actionlib.SimpleActionServer("speak", pollySpeechAction, self._speak_callback, auto_start=False)
        self._speak_server.register_preempt_callback(self._preempt_callback)
        
        self._play_client = SoundClient(blocking=True)

        #start the library
        self._audio_lib = PollyAudioLibrary()

        #start actuin server 
        self._speak_server.start()

        rospy.loginfo("PollyNode ready")

    def _synthesize_speech(self, text, voice_id):
        #Call Amazon Poly and try to generate the speech
        try:
            if text.startswith('<speak>'):
                response = self._polly.synthesize_speech(Text=text, OutputFormat='pcm', VoiceId=voice_id, TextType='ssml')
            else:
                response = self._polly.synthesize_speech(Text=text, OutputFormat='pcm', VoiceId=voice_id)
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
        self._play_client.stopAll()

    def _speak_callback(self, goal):

        complete = True
        rospy.loginfo('POLLY_SPEAK:{}'.format(goal.text))
        if not self._no_audio_flag:
            complete = self.speak(goal)
        result = pollySpeechResult()
        result.complete = complete
        #check if speak failed because it is being preempted
        if not complete and self._speak_server.is_preempt_requested():
            rospy.loginfo('set aborted')
            self._speak_server.set_aborted(result)
        else:
            self._speak_server.set_succeeded(result)


    def speak(self, goal):

        data = None

        # try finding the text in the local stored library
        wav_path = self._audio_lib.find_text(goal.text, goal.voice_id)

        # synthesize speech if doesn't exist
        if wav_path is None:
            rospy.loginfo("synthesizing speech with AWS")
            data = self._synthesize_speech(goal.text, goal.voice_id)
            if data is None :
                #this means that for some reason it wasn't able to generate the speech
                #return failure
                return False
            #check if server is preempted
            if self._speak_server.is_preempt_requested():
                return False

            #save the data into a wav file
            wav_path = self._audio_lib.save_text(goal.text, goal.voice_id, data)
            if wav_path is None:
                #this probably means there is a conflict, which is nearly impossible
                raise RuntimeError
        
        #play the wave file
        self._play_client.playWave(wav_path)
        #check whether we were pre-empted
        if self._speak_server.is_preempt_requested():
            return False
        return True


if __name__ == '__main__':
    rospy.init_node("polly_node")
    pl = PollyNode()
    rospy.spin()
