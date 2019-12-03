#! /usr/bin/env python
import matplotlib.pyplot as plt
from scipy.fftpack import fft
from scipy.io import wavfile # get the api
from scipy.signal import stft
from scipy.signal import fftconvolve, butter
import numpy as np
from scipy import signal

import pyaudio

select = 1

if select == 0:

    CHUNK = 4096 # number of data points to read at a time
    RATE = 44100 # time resolution of the recording device (Hz)

    p=pyaudio.PyAudio() # start the PyAudio class
    stream=p.open(format=pyaudio.paInt16,channels=1,rate=RATE,input=True,
                  frames_per_buffer=CHUNK) #uses default input device


    data_buffer = np.array([])
    # create a numpy array holding a single read of audio data
    for i in range(30): #to it a few times just to see
        print(i)
        data = np.frombuffer(stream.read(CHUNK),dtype=np.int16)
        data_buffer = np.concatenate([data_buffer, data])

        sos = signal.butter(10, 15, 'hp', fs=1000, output='sos')
        data_buffer = signal.sosfilt(sos, data_buffer)


    # close the stream gracefully
    stream.stop_stream()
    stream.close()
    p.terminate()

    np.save('ding69.npy', data_buffer)
    plt.plot(data_buffer)
    plt.show()

else:

    data_buffer = np.load('ding69.npy')[74700: 107700]
    np.save('ding_select_floor2_left_mic.npy', data_buffer)
    plt.plot(data_buffer)
    plt.show()

