# -*- coding: utf-8 -*-
"""
Created on Mon Feb 25 22:34:39 2019

@author: strai
"""
import matplotlib.pyplot as plt
import numpy as np

"""
Fs = 8000
f = 261
sample = 1600
x = np.arange(sample)
y = 2054 + 2054 * np.sin(2 * np.pi * f * x / Fs)
"""

#import pyaudio


#p = pyaudio.PyAudio()

volume = 0.5     # range [0.0, 1.0]
fs = 20000      # sampling rate, Hz, must be integer
duration = 1   # in seconds, may be float
f = 20   # sine frequency, Hz, may be float

# generate samples, note conversion to float32 array
samples = (1024+1024*np.sin(2*np.pi*np.arange(fs*duration)*f/fs)).astype(np.float32)
#samples = (1024+1024*np.sin(2*np.pi*np.arange(fs*duration)*f/fs)).astype(np.float32)

# for paFloat32 sample values must be in range [-1.0, 1.0]
"""
stream = p.open(format=pyaudio.paFloat32,
                channels=1,
                rate=fs,
                output=True)

# play. May repeat with different volume values (if done interactively) 
stream.write(volume*samples)

stream.stop_stream()
stream.close()

p.terminate()"""
x = np.arange(1000)
plt.plot(x, samples[:1000])
plt.xlabel('sample(n)')
plt.ylabel('voltage(V)')
plt.show()


for sample in samples[:1000]:
    print(sample.astype(np.int), end=", ")