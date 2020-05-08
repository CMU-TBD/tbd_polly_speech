
# Changelog

## [1.1.2]
#### Changed
- Removed the namespace tag, let programs decide what ns to put the code in.
#### Fixed
- Fixed where we forgot to change the python_src name to `tbd`

## [1.1.1] - 2020-04-06
* Fixed where we forgot to change the python_src name to `tbd`
* Exposed whether the `pollyspeech.wait` method timed out or not
## [1.1.0] - 2020-04-04
* moved a cloned fork back to `cmu-tbd`. Changed source of action definition to `tbd_ros_msgs`
* Fixed python2 bugs (invalid writing, etc)
## [1.0.0] - 2019-12-04
* change the default output/play_type to be `sound_play`
* fixed a bug where `sound_play` will never be used if start through launch file
* Imporved documentation.