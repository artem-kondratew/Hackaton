import subprocess
import psutil
import sys

date = "date"
date_output = subprocess.check_output(date)


def find_remote_ip() -> str:
    output = subprocess.check_output('echo $SSH_CLIENT', shell=True).decode("utf-8")[:-1]
    return output[:output.find(" ")]


def main():
    print("Video_started at: ", date_output.decode("utf-8"))
    
    ip = find_remote_ip()
    if ip == '':
        print('no remote ip')
        sys.exit(-1)
    
    gs = f'gst-launch-1.0 v4l2src device=/dev/video0 ! video/x-raw, format=I420, width=320, height=240, framerate=30/1 ! rtpvrawpay ! udpsink host={ip} port=5000'
    
    subprocess.run(gs, shell=True, stdin=subprocess.PIPE)
