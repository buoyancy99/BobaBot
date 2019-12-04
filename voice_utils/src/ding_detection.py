#! /usr/bin/env python
import matplotlib.pyplot as plt
from scipy.fftpack import fft
from scipy.io import wavfile # get the api
from scipy.signal import stft
from scipy.signal import fftconvolve
import numpy as np
import pyaudio
import rospy
import sys
from std_msgs.msg import Int8

def detection(ding_left, ding_right, is_7=True):
	pub = rospy.Publisher('ding_value', Int8, queue_size=10)
	rospy.init_node('ding_detector')
	rate = rospy.Rate(10)

	CHUNK = 4096 # number of data points to read at a time
	RATE = 44100 # time resolution of the recording device (Hz)

	p=pyaudio.PyAudio() # start the PyAudio class

	stream=p.open(format=pyaudio.paInt16,channels=1,rate=RATE,input=True,
		              frames_per_buffer=CHUNK) #uses default input device

	while not rospy.is_shutdown():
		

		data_buffer = np.array([])
		# create a numpy array holding a single read of audio data
		for i in range(10): #to it a few times just to see
		    data = np.frombuffer(stream.read(CHUNK),dtype=np.int16)
		    data_buffer = np.concatenate([data_buffer, data])

		d_left = fftconvolve(ding_left, data_buffer)
		d_right = fftconvolve(ding_right, data_buffer)
		dlmax = d_left.max()
		drmax = d_right.max()
		#print("left ding is:" +str(dlmax))
		#print("right ding is:" +str(drmax))
		#FLOOR 7

		if is_7:
			l_threshold = 20173224741.999992
			r_threshold = 30888468567.000004
		else:
			l_threshold = 10008361056.999992
			r_threshold = 2000511377.789566


		if dlmax > l_threshold:
			print('Left DING')
			pub.publish(1)
		elif drmax > r_threshold:
			print('Right DING')
			pub.publish(2)
		else:
			pub.publish(0)


	stream.stop_stream()
	stream.close()
	p.terminate()

if __name__ == "__main__":
	if sys.argv[1] == '7':
		ding_left = np.load('/home/rmcal/Projects/BobaBot/src/voice_utils/src/ding_select_floor7_left_mic.npy')
		ding_right = np.load('/home/rmcal/Projects/BobaBot/src/voice_utils/src/ding_select_floor7_right_mic.npy')
		is_7 = True
	else:
		ding_left = np.load('/home/rmcal/Projects/BobaBot/src/voice_utils/src/ding_select_floor2_left_mic.npy')
		ding_right = np.load('/home/rmcal/Projects/BobaBot/src/voice_utils/src/ding_select_floor2_right_mic.npy')
		is_7 = False
	detection(ding_left, ding_right, is_7)

# data_buffer = np.load('ding2.npy')[73000: 130000]

# np.save('ding_select.npy', data_buffer)
# plt.plot(data_buffer)
# plt.show()
# d = fftconvolve(a, data)
# plt.plot(d)
# print(d.max())
# plt.show()

# close the stream gracefully
	
