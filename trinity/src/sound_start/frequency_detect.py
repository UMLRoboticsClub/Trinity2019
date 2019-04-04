#!/usr/bin/env python3
import pyaudio
import wave
import numpy as np
import signal
import sys

# Configuration:
target_freq = 3800
#+/-Hz
tolerance = 20
#how many times do we have to get the
#frequency for it to be accepted
freq_duration = 5

# Probably don't need to mess with this stuff:
sample_rate = 44100
chunk_size = 1024
audio_format = pyaudio.paInt16
num_channels = 1

#
def signal_handler(sig, frame):
        print('\nAborting...')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

def wait_for_freq():
    window = np.blackman(chunk_size)

    p = pyaudio.PyAudio()

    stream = p.open(
            format=audio_format,
            channels=num_channels,
            rate=sample_rate,
            input=True,
            frames_per_buffer=chunk_size)

    count = 0
    while(True):
        data = stream.read(chunk_size)

        swidth = p.get_sample_size(audio_format)
        indata = np.array(wave.struct.unpack("%dh"%(len(data)/swidth), data))*window
        fftData = abs(np.fft.rfft(indata))**2
        which = fftData[1:].argmax() + 1
        if which != len(fftData)-1:
            y0,y1,y2 = np.log(fftData[which-1:which+2:])
            x1 = (y2 - y0) * .5 / (2 * y1 - y2 - y0)
            freq = (which+x1)*sample_rate/chunk_size
        else:
            freq = which*sample_rate/chunk_size

        #print(freq)
        if freq > (target_freq - tolerance) and freq < (target_freq + tolerance):
            count += 1 
        else:
           count = 0

        if count == freq_duration:
            return
