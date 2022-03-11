import io
import socket
import struct
import time
import picamera
import os
import pandas as pd
import subprocess
from fractions import Fraction
var = subprocess.check_output('pgrep ptpd', shell=True)
pid = var.decode("utf-8")

class SplitFrames(object):
    def __init__(self, connection):
        self.connection = connection
        self.df = pd.DataFrame(columns=['ts'])
        self.stream = io.BytesIO()
        self.count = 0

    def write(self, buf):
        if buf.startswith(b'\xff\xd8'):
            # Start of new frame; send the old one's length
            # then the data
            size = self.stream.tell()
            if size > 0:
                self.connection.write(struct.pack('<L', size))
                self.connection.flush()
                self.stream.seek(0)
                self.connection.write(self.stream.read(size))
                self.df.loc[len(self.df.index)] = [time.time_ns()]
                self.count += 1
                self.stream.seek(0)
        self.stream.write(buf)

serverAdressPort = ("192.168.0.100", 8888)
bufferSize = 1024
UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

client_socket = socket.socket()
client_socket.connect(('192.168.0.103', 8000))
connection = client_socket.makefile('wb')
print("[INFO] connected to central...")

try:
    with picamera.PiCamera(resolution=(640,480), framerate=90,
                       sensor_mode=7) as camera:
        #camera.start_preview(fullscreen=False,window=(100,100,640,480))
        print("[INFO] setting up camera")
        camera.exposure_mode = 'off'    
        camera.awb_mode = 'off'
        camera.awb_gains = (Fraction(331, 256), Fraction(203, 128))
        camera.analog_gain=Fraction(8)
        camera.digital_gain=Fraction(383,256)
        camera.shutter_speed = camera.exposure_speed
        camera.color_effects = (128,128)
        output = SplitFrames(connection)
        
        print("[INFO] triggering server")
        UDPClientSocket.sendto(str.encode("on"),serverAdressPort)
        msgFromServer = UDPClientSocket.recvfrom(bufferSize)
        data = msgFromServer[0].split()
        timeBase = int(data[0])
        trigger = int(data[1])
        recTime = int(data[2])

        os.system('sudo kill -9 '+pid)
        print('[INFO] killed PTPD process')
        
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
        # Write the terminating 0-length to the connection to let the
        # server know we're done
        connection.write(struct.pack('<L', 0))
finally:
    connection.close()
    client_socket.close()
    finish = time.time()
print('Sent %d images in %d seconds at %.2ffps' % (
    output.count, finish-start, output.count / (finish-start)))
output.df.to_csv('results_1.csv', index = False)
print('[RESULTS] csv exported with '+str(len(output.df.index))+' lines')
os.system('sudo ptpd -s -i eth0')
print('[INFO] PTPD running')
os.system('sshpass -p "debora123#" scp results_1.csv debora@192.168.0.103:~/Desktop/MoCapRasps/results')
print('[INFO] csv sent to central')
os.system('rm -rf results_1.csv')