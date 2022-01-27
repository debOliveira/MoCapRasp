import io,socket
import time
import picamera
import os
import pandas as pd
os.system('rm -rf pics/*')

trigger = 5*(10**9) #miliseconds
recTime = 125
print('[INFO] set trigger to '+ str(trigger/(10**9)) + 's '+ 'and recording time to '+ str(recTime) + 's')

class SplitFrames(object):
    def __init__(self):
        self.frame_num = 0
        self.df = pd.DataFrame(columns=['timestamp(microsec)'])
        self.output = None

    def write(self, buf):
        if buf.startswith(b'\xff\xd8'):
            # Start of new frame; close the old one (if any) and
            # open a new output
            if self.output:
                self.output.close()
            self.frame_num += 1
            self.output = io.open('pics/image%04d.jpg' % self.frame_num, 'wb')
            self.df.loc[len(self.df.index)] = [time.time_ns()]
        self.output.write(buf)
        
localIp = "192.168.0.103"
localPort = 8888
bufferSize = 1024
UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
UDPServerSocket.bind((localIp,localPort))
print("[INFO] server running...")

with picamera.PiCamera(resolution=(640,480), framerate=40,
                       sensor_mode=4) as camera:
    camera.start_preview(fullscreen=False,window=(100,100,640,480))
    print("[INFO] setting up camera")
    # Give the camera some warm-up time
    time.sleep(5)
    camera.shutter_speed = camera.exposure_speed
    camera.exposure_mode = 'off'
    camera.color_effects = (128,128)
    g = camera.awb_gains
    camera.awb_mode = 'off'
    camera.awb_gains = g
    output = SplitFrames()
    
    print("[INFO] waiting for client")
    bytesPair = UDPServerSocket.recvfrom(bufferSize)
    adress = bytesPair[1]
    timeBase = time.time_ns()
    output.tb = timeBase
    print(str(adress) + ' >> '+ str(int(timeBase)/(10**9)))
    UDPServerSocket.sendto(str.encode(str(timeBase)+' '+str(trigger)+' '+str(recTime)),adress)
    
    print("[INFO] waiting trigger")
    now = time.time_ns()
    while (now - timeBase) < (trigger):
        now = time.time_ns()
    
    print('[RECORDING..]')
    start = time.time()
    camera.start_recording(output, format='mjpeg')
    camera.wait_recording(recTime)
    camera.stop_recording()
    finish = time.time()
    
print('Captured %d frames at %.2ffps' % (
    output.frame_num,
    output.frame_num / (finish - start)))
output.df.to_csv('results.csv', index = False)
print('[RESULTS] csv exported with '+str(len(output.df.index))+' lines')