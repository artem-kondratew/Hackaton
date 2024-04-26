import os

video = 'gst-launch-1.0 udpsrc port="5000" caps="application/x-rtp, media=(string)video, encoding-name=(string)RAW, sampling=(string)YCbCr-4:2:0, width=(string)640, height=(string)480, framerate=30/1" ! rtpvrawdepay ! videoconvert ! autovideosink'

def main():
    os.system(video)
