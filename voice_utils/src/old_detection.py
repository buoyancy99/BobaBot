#! /usr/bin/env python
import matplotlib.pyplot as plt
from scipy.fftpack import fft
from scipy.io import wavfile # get the api
from scipy.signal import fftconvolve, convolve, stft, butter
import numpy as np
from scipy import signal
import warnings
warnings.filterwarnings("ignore")

from numpy import array, diff, where, split
from scipy import arange


# fs, data = wavfile.read('Ding.wav') # load OG file
# a = data.T[0] 

# fs2, data2 = wavfile.read('Long.wav') # load the data
# a2 = data2.T[0]

# d = fftconvolve(a, a2)
# print(d.shape)
# for i in range(len(d)):
# 	if d[i] > 0.85: #Tune this for the DING
# 		print('Do something')
# 		break
# plt.plot(d)
# plt.show()

#import keyboard

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

def extractFrequency(indices, freq_threshold=2):
    
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


import pyaudio

ding_left = np.load('ding_select_floor2_left_mic.npy')
ding_right = np.load('ding_select_floor7_right_mic.npy')

CHUNK = 4096 # number of data points to read at a time
RATE = 48000 # time resolution of the recording device (Hz)

p=pyaudio.PyAudio() # start the PyAudio class
stream=p.open(format=pyaudio.paInt16,channels=1,rate=RATE,input=True,
              frames_per_buffer=CHUNK) #uses default input device

while 1:
	data_buffer = np.array([])
	# create a numpy array holding a single read of audio data
	for i in range(10): #to it a few times just to see
	    data = np.frombuffer(stream.read(CHUNK),dtype=np.int16)
	    data_buffer = np.concatenate([data_buffer, data])

	fs = RATE
	# f1, t1, ding_left2 = signal.stft(ding_left, fs, nperseg=1000)
	# f2, t2, ding_right2 = signal.stft(ding_right, fs, nperseg=1000)
	# f,t,data_buffer2= signal.stft(data_buffer, fs, nperseg=1000)

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
	    
	indices = findPeak(magnitude_values=magnitude_values, noise_level=200)
	frequencies = extractFrequency(indices=indices)

	#print(frequencies)

	# amp = 2 * np.sqrt(2)
	# plt.pcolormesh(t1, f1, np.abs(ding_left), vmin=0)
	# plt.pcolormesh(t2, f2, np.abs(ding_right), vmin=0)
	# plt.ylabel('Frequency [Hz]')
	# plt.xlabel('Time [sec]')
	# plt.show()
	#x = np.abs(data_buffer2).mean()
	x = max(frequencies)
	#x = x/1000
	print(x)
	if x > 730 and x < 800:
		print("RIGHT DING MAYBE")
	# if x > 270 and x < 350:
	# 	print("LEFT DING MAYBE")
	# if x > 1300 and x < 1400:
	# 	print("RIGHT DING MAYBE")
	# if x > 500 and x < 550:
	# 	print("LEFT DING MAYBE")

	

	#print(np.abs(data_buffer).max())
	# d_left = convolve(ding_left, data_buffer)
	# d_right = convolve(ding_right, data_buffer)
	

	# dlmax = d_left.mean()
	# drmax = d_right.mean()
	#print("left ding is:" +str(dlmax) + "right ding is:" +str(drmax))
	#print("right new is:" + str(d_right_fft.mean()))
	#FLOOR 7
	# if dlmax > 20173224741.999992:
	# 	print('Left DING')
	# if drmax > 30888468567.000004:
	# 	print('Right DING')
	# if dlmax > 10008361056.999992:
	# 	print('Left DING')
	# if drmax > 2000511377.789566:


	# 	print('Right DING')
	



# data_buffer = np.load('ding2.npy')[73000: 130000]

# np.save('ding_select.npy', data_buffer)
# plt.plot(data_buffer)
# plt.show()
# d = fftconvolve(a, data)
# plt.plot(d)
# print(d.max())
# plt.show()

# close the stream gracefully
stream.stop_stream()
stream.close()
p.terminate()