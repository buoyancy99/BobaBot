#!/usr/bin/env python
import speech_recognition as sr
import pygame as pg
from playsound import playsound
from voice_info.srv import talk, talkResponse
#import rospy, os, sys

#import roslib; roslib.load_manifest('sound_yak')



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
    #Fit this stuff, kinda hardcode it for the responses. 
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

def handle_talk(req):
    recognizer = sr.Recognizer()
    mic = sr.Microphone() #selects default, change to use 
    response = recognize_speech_from_mic(recognizer, mic)
    
    print('\nSuccess : {}\nError   : {}\n\nText from Speech\n{}\n\n{}' \
          .format(response['success'],
                  response['error'],
                  '-'*17,
                  response['transcription']))
    call(response['transcription'])
    return talkResponse(1)


def talk_server():
    rospy.init_node('talk_server')
    s = rospy.Service('talk', talk, handle_talk)
    rospy.spin()

if __name__ == "__main__":
    talk_server()

### Client function
def talk_client:
    rospy.wait_for_service('talk')
    try:
        talk_func = rospy.ServiceProxy('talk', talk)
        resp1 = talk_func(1)
        return resp1.result
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e