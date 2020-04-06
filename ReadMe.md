# tbd_polly_speech
License - MIT  
Maintainer - zhi.tan@ri.cmu.edu  

A ROS wrapper for Amazon Polly Text-to-Speech service. We also cache locally each soundfile, so if you ever repeat the same sentence with the same voice, a local copy of the audio will be used instead of sythesizing it again.

## Dependencies
### ROS Packages:
* tbd_ros_msgs (https://github.com/CMU-TBD/tbd_ros_msgs)
* sound_play (http://wiki.ros.org/sound_play)
* IF uses `tbd_audio_common` play type:
    * tbd_ros_msgs (https://github.com/CMU-TBD/tbd_ros_msgs)
    * tbd_audio_common (https://github.com/CMU-TBD/tbd_audio_common)
### Python Dependencies
* boto3

## Usage

### Running 
1. Make sure you have a AWS credentials file setup on the system. [Guide by AWS](https://docs.aws.amazon.com/cli/latest/userguide/cli-config-files.html). Make sure the account has AWS Polly Speech Enabled
2. launch the backend services
```
roslaunch tbd_polly_speech polly_speech.launch
```
3. You can access the service either through the Python API
```
from succes_polly_speech import PollySpeech

ps = PollySpeech()

ps.speak("I am a good robot",voice_id='Joanna')
ps.speak("I am not a scary robot",voice_id='Joanna', block=False)
ps.wait()
ps.speak('Hello World. I will be interrupted',voice_id='Emma', block=False, cancel=True) 
ps.stopAll() #Interrupts the sentence and stop the voice command
```
OR directly calling the action server at the topic `tbd_polly_speech/speak` with the action `pollySpeechAction`.

### ROS Parameters
There are three ROS parameters in the launch file
* `no_audio`, true if you just want to simulate it and not actually running the code. 
* `play_type`, the default is `sound_play` in the `ros-driver/audio_common` repository. You can install this with `sudo apt install ros-melodic-audio-common`. The alternative is TBD's lab own audio stack (`tbd_audio_common`) that uses actionlib instead of ROS messages and plays faster.

* `polly_audio_storage_path`, the path to the location you want to store the audio and also the `masterlist.txt` which stores the coding from phrases/text to filename. the default is `PACKAGE_ROOT/audio_storage`

### Voices
A list of voice ID can be found here: https://docs.aws.amazon.com/polly/latest/dg/voicelist.html

## Change logs:
* 4/6/2020:
    * Fixed where we forgot to change the python_src name to `tbd`
    * Exposed whether the `pollyspeech.wait` method timed out or not
* 4/4/2020:
    * moved a cloned fork back to `cmu-tbd`. Changed source of action definition to `tbd_ros_msgs`
    * Fixed python2 bugs (invalid writing, etc)
* 12/4/2019
    * change the default output/play_type to be `sound_play`
    * fixed a bug where `sound_play` will never be used if start through launch file
    * Imporved documentation.

## Past Contributors
- Joe Connolly - 07/2018