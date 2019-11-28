import matplotlib.pyplot as plt
from scipy.fftpack import fft
from scipy.io import wavfile # get the api
from scipy.signal import stft
from scipy.signal import fftconvolve
import numpy as np


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
import pyaudio

ding = np.load('ding_select_floor7_left.npy')

CHUNK = 4096 # number of data points to read at a time
RATE = 44100 # time resolution of the recording device (Hz)

p=pyaudio.PyAudio() # start the PyAudio class
stream=p.open(format=pyaudio.paInt16,channels=1,rate=RATE,input=True,
              frames_per_buffer=CHUNK) #uses default input device

for _ in range(30):
	data_buffer = np.array([])
	# create a numpy array holding a single read of audio data
	for i in range(10): #to it a few times just to see
	    data = np.frombuffer(stream.read(CHUNK),dtype=np.int16)
	    data_buffer = np.concatenate([data_buffer, data])

	d = fftconvolve(ding, data_buffer)
	dmax = d.max()
	print(dmax)
	if dmax > 200000000:
		print('Ding!')



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