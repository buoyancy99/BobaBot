#!/bin/bash
roslaunch audio_capture capture.launch #Launch don't close
rostopic hz /audio #Launch to stream audio

roslaunch audio_play play.launch #Plays incoming messages