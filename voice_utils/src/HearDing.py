#! /usr/bin/env python
import matplotlib.pyplot as plt
from scipy.fftpack import fft
from scipy.io import wavfile # get the api
from scipy.signal import stft
from scipy.signal import fftconvolve
import numpy as np


fs, data = wavfile.read('Ding.wav') # load OG file
a = data.T[0] 

fs2, data2 = wavfile.read('Short.wav') # load the data
a2 = data2.T[0]

d = fftconvolve(a, a2)
print(d.shape)
for i in range(len(d)):
	if d[i] > 0.85: #Tune this for the DING
		print('Do something')
		break
plt.plot(d)
plt.show()