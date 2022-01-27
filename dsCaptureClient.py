import io,socket
import time
import picamera
import os
import pandas as pd
os.system('rm -rf pics/*')

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
        
serverAdressPort = ("192.168.0.103", 8888)
bufferSize = 1024
UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

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
    
    print("[INFO] triggering server")
    UDPClientSocket.sendto(str.encode("on"),serverAdressPort)
    msgFromServer = UDPClientSocket.recvfrom(bufferSize)
    data = msgFromServer[0].split()
    timeBase = int(data[0])
    trigger = int(data[1])
    recTime = int(data[2])
        
    print("[INFO] waiting camera trigger")
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