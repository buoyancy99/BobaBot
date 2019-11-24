import speech_recognition as sr
import pygame as pg
from playsound import playsound

import roslib; roslib.load_manifest('sound_yak')
import rospy, os, sys
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from sound_yak.msg import yak_cmd

def recognize_speech_from_mic(recognizer, microphone):
    if not isinstance(recognizer, sr.Recognizer):
        raise TypeError("`recognizer` must be `Recognizer` instance")

    if not isinstance(microphone, sr.Microphone):
        raise TypeError("`microphone` must be `Microphone` instance")

    with microphone as source:
        recognizer.adjust_for_ambient_noise(source)
        audio = recognizer.listen(source)

    # set up the response object
    response = {
        "success": True,
        "error": None,
        "transcription": None
    }

    try:
        response["transcription"] = recognizer.recognize_google(audio)
    except sr.RequestError:
        # API was unreachable or unresponsive
        response["success"] = False
        response["error"] = "API unavailable/unresponsive"
    except sr.UnknownValueError:
        # speech was unintelligible
        response["error"] = "Unable to recognize speech"

    return response

def call(response):
    response = response.lower()
    if 'hello' in response or 'hi' in response:
        #print('Identified Boba, play respective recording ')
       	playsound("audio_recordings/order_boba.mp3")
    if 'all' in response or 'that' in response:
        #print('Identified Boba, play respective recording ')
       	playsound("audio_recordings/ask_cost.mp3")
    if 'your' in response or 'here' in response:
        #print('Identified Boba, play respective recording ')
       	playsound("audio_recordings/boba_put.mp3")
    if 'thank' in response or 'you' in response:
        #print('Identified Boba, play respective recording ')
       	playsound("audio_recordings/thank_end.mp3")
    return 

if __name__ == "__main__":

    recognizer = sr.Recognizer()
    mic = sr.Microphone() #selects default, change to use 
    response = recognize_speech_from_mic(recognizer, mic)
    
    print('\nSuccess : {}\nError   : {}\n\nText from Speech\n{}\n\n{}' \
          .format(response['success'],
                  response['error'],
                  '-'*17,
                  response['transcription']))
    call(response['transcription'])

#!/usr/bin/env python


# directory with sound assets - change as needed
soundAssets = '/home/shiloh/devel/audio_assets/'
# duration of yak throttle
throttle = 3 # seconds

def sound_translator(data):
    print data
    global allow_yak
    if rospy.Time.now() <= allow_yak: # Throttles yak to avoid
        print("Sound throttled")      # SoundClient segfault
        return
    # when to reallow yak
    allow_yak = rospy.Time.now() + rospy.Duration.from_sec(throttle)
    if data.cmd == "wav":
        soundhandle.playWave(soundAssets + data.param)
    if data.cmd == "say":
        soundhandle.say(data.param)

def yak_init():
    rospy.init_node('yak_node', anonymous = True)
    global allow_yak
    allow_yak = rospy.Time.now()
    rospy.Subscriber('yak', yak_cmd, sound_translator)
    rospy.spin()

if __name__ == '__main__':
    soundhandle = SoundClient()
    rospy.sleep(1)
    
    yak_init()


