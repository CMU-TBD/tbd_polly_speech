
# Changelog

## [1.2.1] - 2021-04-07
- **[Fixed]** bug where hashes were refreshed every session. Use `hashlib` now.
- **[Added]** some 

## [1.2.0] - 2021-03-17
- **[Changed]** reformatting of files
- **[Changed]** renamed masterfile to hash_table and change it to use yaml instead of pure txt file.
- **[Fixed]** bug where index file kept being recreated.

## [1.1.3] - 2020-07-01
#### Added
- CI Test that checks the interfaces
- `speak_signal` that sends a `std_msgs/Bool` at 20Hz to signal whether the robot is speaking or not.

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