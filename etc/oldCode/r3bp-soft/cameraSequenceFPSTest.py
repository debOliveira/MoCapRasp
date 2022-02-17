import io,socket
import time
import picamera

class SplitFrames(object):
    def __init__(self):
        self.frame_num = 0
        self.output = None

    def write(self, buf):
        if buf.startswith(b'\xff\xd8'):
            # Start of new frame; close the old one (if any) and
            # open a new output
            if self.output:
                self.output.close()
            self.frame_num += 1
            self.output = io.open('pics/image%02d.jpg' % self.frame_num, 'wb')
        self.output.write(buf)
            
localIp = "192.168.1.103"
localPort = 8888
bufferSize = 1024
UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
UDPServerSocket.bind((localIp,localPort))
print("[INFO] server running...")

with picamera.PiCamera(resolution=(640,480), framerate=90,
                       sensor_mode=7) as camera:
    # Give the camera some warm-up time
    time.sleep(2)
    camera.shutter_speed = camera.exposure_speed
    camera.exposure_mode = 'off'
    camera.awb_mode = 'off'
    camera.awb_gains = 0.9
    output = SplitFrames()
    
    bytesPair = UDPServerSocket.recvfrom(bufferSize)
    adress = bytesPair[1]
    timeBase = time.time_ns()
    print(str(adress) + ' >> '+ str(int(timeBase)/(10**9)))
    UDPServerSocket.sendto(str.encode(str(timeBase)),adress)
    now = time.time_ns()
    while (now - timeBase) < (10**8):
        now = time.time_ns()
    print((now - timeBase)/(10**9),now/(10**9))
        
    start = time.time()
    camera.start_recording(output, format='mjpeg')
    camera.wait_recording(2)
    camera.stop_recording()
    finish = time.time()
    
print('Captured %d frames at %.2ffps' % (
    output.frame_num,
    output.frame_num / (finish - start)))