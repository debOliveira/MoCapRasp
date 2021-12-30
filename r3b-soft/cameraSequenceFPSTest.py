import io,socket
import time
import picamera
import os
import pandas as pd

os.system('clear')
os.system('rm -rf pics/*')

trigger = 10**9 #miliseconds
recTime = 2
print('[INFO] set trigger to '+
      str(trigger/(10**9)) + 's '+
      'and recording time to '+
      str(recTime) + 's')

class SplitFrames(object):
    def __init__(self):
        self.frame_num = 0
        self.output = None
        self.df = pd.DataFrame(columns=['timestamp(microsec)'])
        self.tb = 0

    def write(self, buf):
        if buf.startswith(b'\xff\xd8'):
            # Start of new frame; close the old one (if any) and
            # open a new output
            if self.output:
                self.output.close()
            self.frame_num += 1
            self.output = io.open('pics/image%05d.bmp' % self.frame_num, 'wb')
            self.df.loc[len(self.df.index)] = [time.time_ns()]
        self.output.write(buf)
    
            
serverAdressPort = ("192.168.1.104", 8888)
bufferSize = 1024
UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

print("[INFO] setting up camera")
with picamera.PiCamera(resolution=(640,480), framerate=70,
                       sensor_mode=7) as camera:
    # Give the camera some warm-up time
    time.sleep(2)
    camera.shutter_speed = camera.exposure_speed
    camera.exposure_mode = 'off'
    camera.awb_mode = 'off'
    camera.awb_gains = 0.9
    output = SplitFrames()
    
    print("[INFO] waiting for client")
    UDPClientSocket.sendto(str.encode("on"),serverAdressPort)
    msgFromServer = UDPClientSocket.recvfrom(bufferSize)
    data = msgFromServer[0].split()
    timeBase = int(data[0])
    trigger = int(data[1])
    recTime = int(data[2])
    
    
    print("[INFO] waiting trigger")
    now = time.time_ns()
    while (now - timeBase) < (trigger):
        now = time.time_ns()
    
    print('[RECORDING]')
    start = time.time()
    camera.start_recording(output, format='mjpeg')
    camera.wait_recording(recTime)
    camera.stop_recording()
    finish = time.time()

print('[RESULTS] waited '+ str((now - timeBase)/(10**9)) + 's')
print('[RESULTS] trigger at timestamp ' +str(now/(10**9)))
    
print('[RESULTS] captured %d frames at %.2ffps' % (
    output.frame_num,
    output.frame_num / (finish - start)))
output.df.loc[len(output.df.index)] = [output.tb]
output.df.to_csv('results.csv', index = False)
print('[RESULTS] csv exported with '+str(len(output.df.index))+' lines')
