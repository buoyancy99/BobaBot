import matplotlib.pyplot as plt
from scipy.fftpack import fft
from scipy.io import wavfile # get the api
from scipy.signal import stft
from scipy.signal import fftconvolve
import numpy as np
import pyaudio
import rospy


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

def detection():
	pub = rospy.Publisher('ding_detect', Int8, queue_size=10)
	rospy.init_node('ding_detector')
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		CHUNK = 4096 # number of data points to read at a time
		RATE = 44100 # time resolution of the recording device (Hz)

		p=pyaudio.PyAudio() # start the PyAudio class
		stream=p.open(format=pyaudio.paInt16,channels=1,rate=RATE,input=True,
		              frames_per_buffer=CHUNK) #uses default input device

		data_buffer = np.array([])
		# create a numpy array holding a single read of audio data
		for i in range(10): #to it a few times just to see
		    data = np.frombuffer(stream.read(CHUNK),dtype=np.int16)
		    data_buffer = np.concatenate([data_buffer, data])

		d = fftconvolve(ding, data_buffer)
		dmax = d.max()

		print(dmax)
		if dmax > 700000000:
			print('DING')
			pub.publish(1)
		else:
			pub.publish(0)
		stream.stop_stream()
		stream.close()
		p.terminate()

if __name__ == "__main__":
	ding = np.load('ding_select_floor7_left.npy')
	detection()

# data_buffer = np.load('ding2.npy')[73000: 130000]

# np.save('ding_select.npy', data_buffer)
# plt.plot(data_buffer)
# plt.show()
# d = fftconvolve(a, data)
# plt.plot(d)
# print(d.max())
# plt.show()

# close the stream gracefully
	