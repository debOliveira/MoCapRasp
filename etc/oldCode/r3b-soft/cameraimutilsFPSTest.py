from imutils.video.pivideostream import PiVideoStream
import imutils
import io,socket
import time
import os
import pandas as pd
import numpy as np

#os.system('clear')
os.system('rm -rf pics/*')

count = 0            
serverAdressPort = ("192.168.1.104", 8888)
bufferSize = 1024
UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
df = pd.DataFrame(columns=['timestamp(microsec)','image','diff2base(sec)'])
camera = PiVideoStream(resolution=(640,480), framerate=70,).start()

print("[INFO] setting up camera")
# Give the camera some warm-up time
time.sleep(2)
    
print("[INFO] triggering server")
UDPClientSocket.sendto(str.encode("on"),serverAdressPort)
msgFromServer = UDPClientSocket.recvfrom(bufferSize)
data = msgFromServer[0].split()
timeBase = int(data[0])
trigger = int(data[1])
frameTotal = int(data[2])
interval = int(data[3])
df.loc[len(df.index)] = [timeBase,np.zeros((640,480)),0]
    
print("[INFO] waiting camera trigger")
now = time.time_ns()
while (now - timeBase) < (trigger):
    now = time.time_ns()
print('[RESULTS] waited '+ str((now - timeBase)/(10**9)) + 's')
print('[RESULTS] trigger at timestamp ' +str(now/(10**9)))

timeBase += trigger
print('[RECORDING]')
start = time.time()
while count < frameTotal:
    while (now - timeBase) < (interval):
        now = time.time_ns()
    frame = camera.read()
    df.loc[len(df.index)] = [now,frame,(now-timeBase)/(10**9)]
   # print((now-timeBase)/(10**9))
    timeBase += interval
    count += 1
finish = time.time()
    
print('[RESULTS] captured %d frames at %.2ffps' % (
    frameTotal,
    frameTotal / (finish - start)))
df.to_csv('results.csv', index = False)
print('[RESULTS] csv exported with '+str(len(df.index))+' lines')
