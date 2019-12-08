#! /usr/bin/env python
import matplotlib.pyplot as plt
from scipy.fftpack import fft
from scipy.io import wavfile # get the api
from scipy import signal
from scipy.signal import stft
from scipy.signal import fftconvolve, convolve
import numpy as np
import warnings
warnings.filterwarnings("ignore")

from numpy import array, diff, where, split
from scipy import arange

import pyaudio
import rospy
import sys
from std_msgs.msg import Int8


def findPeak(magnitude_values, noise_level=2000):
    
    splitter = 0
    # zero out low values in the magnitude array to remove noise (if any)
    magnitude_values = np.asarray(magnitude_values)        
    low_values_indices = magnitude_values < noise_level  # Where values are low
    magnitude_values[low_values_indices] = 0  # All low values will be zero out
    
    indices = []
    
    flag_start_looking = False
    
    both_ends_indices = []
    
    length = len(magnitude_values)
    for i in range(length):
        if magnitude_values[i] != splitter:
            if not flag_start_looking:
                flag_start_looking = True
                both_ends_indices = [0, 0]
                both_ends_indices[0] = i
        else:
            if flag_start_looking:
                flag_start_looking = False
                both_ends_indices[1] = i
                # add both_ends_indices in to indices
                indices.append(both_ends_indices)
                
    return indices

def extractFrequency(indices, freq_bins, freq_threshold=2):
    
    extracted_freqs = []
    
    for index in indices:
        freqs_range = freq_bins[index[0]: index[1]]
        avg_freq = round(np.average(freqs_range))
        
        if avg_freq not in extracted_freqs:
            extracted_freqs.append(avg_freq)

    # group extracted frequency by nearby=freq_threshold (tolerate gaps=freq_threshold)
    group_similar_values = split(extracted_freqs, where(diff(extracted_freqs) > freq_threshold)[0]+1 )
    
    # calculate the average of similar value
    extracted_freqs = []
    for group in group_similar_values:
        extracted_freqs.append(round(np.average(group)))
    
    #print("freq_components", extracted_freqs)
    return extracted_freqs

def detection(ding_left, ding_right, is_7=True):
	pub = rospy.Publisher('ding_value', Int8, queue_size=10)
	rospy.init_node('ding_detector')
	rate = rospy.Rate(10)

	CHUNK = 4096 # number of data points to read at a time
	RATE = 48000 # time resolution of the recording device (Hz)

	p=pyaudio.PyAudio() # start the PyAudio class

	stream=p.open(format=pyaudio.paInt16,channels=1,rate=RATE,input=True,
		              frames_per_buffer=CHUNK) #uses default input device
	# sos = signal.butter(10, 500, 'hp', fs=RATE, output='sos')
	# ding_left = signal.sosfilt(sos, ding_left)
	# ding_right = signal.sosfilt(sos, ding_right)
	
	while not rospy.is_shutdown():
		
		data_buffer = np.array([])
		# create a numpy array holding a single read of audio data
		for i in range(10): #to it a few times just to see
		    data = np.frombuffer(stream.read(CHUNK),dtype=np.int16)
		    data_buffer = np.concatenate([data_buffer, data])

		number_samples = len(data_buffer)

		freq_bins = arange(number_samples) * RATE/number_samples

		#ding_left2 = fft(ding_left)
		#ding_right2 = fft(ding_right)
		data_buffer_fft = fft(data_buffer)
		#data_buffer_fft = np.fft.fftfreq(len(data_buffer), data_buffer)
		#print(data_buffer2)

		normalization_data = data_buffer_fft/number_samples
		magnitude_values = normalization_data[range(len(data_buffer_fft)//2)]
		magnitude_values = np.abs(magnitude_values)
		    
		indices = findPeak(magnitude_values=magnitude_values, noise_level=50)
		frequencies = extractFrequency(indices, freq_bins)
		x = max(frequencies)
		#x = x/1000
		print(x)
		if(x >= 760 and x <= 770) or (x >= 790 and x <= 800):
			print("DINGGGGGG")
			pub.publish(1)
		else:
			pub.publish(0)


		# data_buffer = signal.sosfilt(sos, data_buffer)

		# d_left = fftconvolve(ding_left, data_buffer)
		# d_right = fftconvolve(ding_right, data_buffer)
		# # d_left = convolve(ding_left, data_buffer)
		# # d_right = convolve(ding_right, data_buffer)
		# dlmax = d_left.max()
		# drmax = d_right.max()
		# #print("left ding is:" +str(dlmax))
		# #print("right ding is:" +str(drmax))
		# #FLOOR 7

		# if is_7:
		# 	l_threshold = 2509281003.7854402
		# 	r_threshold = 10000000000.0            
			
		# else:
		# 	l_threshold = 10008361056.0
		# 	r_threshold = 8500000000.0

		
		# print('left: ', dlmax)
		# print('right: ', drmax)
		# if dlmax > l_threshold:
		# 	print('Left DING')
		# 	pub.publish(1)
		# elif drmax > r_threshold:
		# 	print('Right DING')
		# 	pub.publish(2)
		# else:
		# 	pub.publish(0)


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
	
