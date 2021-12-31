from imutils.video.pivideostream import PiVideoStream
import imutils
import io,socket
import time
import os
import pandas as pd
import numpy as np

os.system('clear')
os.system('rm -rf pics/*')

trigger = 10**9 #miliseconds
frameTotal = 180
interval = 10**7 #miliseconds
count = 0
print('[INFO] set trigger to '+ str(trigger/(10**9)) + 's '+ 'and recording time to '+ str(frameTotal) + ' frames')
            
localIp = "192.168.1.104"
localPort = 8888
bufferSize = 1024
UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
UDPServerSocket.bind((localIp,localPort))
print("[INFO] server running...")

df = pd.DataFrame(columns=['timestamp(microsec)','image'])
vs = PiVideoStream(resolution=(640,480), framerate=70,).start()

print("[INFO] setting up camera")
with vs as camera:
    # Give the camera some warm-up time
    time.sleep(2)
    
    print("[INFO] waiting for client")
    bytesPair = UDPServerSocket.recvfrom(bufferSize)
    adress = bytesPair[1]
    timeBase = time.time_ns()
    print(str(adress) + ' >> '+ str(int(timeBase)/(10**9)))
    UDPServerSocket.sendto(str.encode(str(timeBase)+' '+str(trigger)+' '+str(frameTotal)+' '+str(interval)),adress)
    df.loc[len(df.index)] = [timeBase,np.zeros((640,480))]

    print("[INFO] waiting trigger")
    now = time.time_ns()
    while (now - timeBase) < (trigger):
        now = time.time_ns()
    
    print('[RECORDING]')
    start = time.time()
    while count < frameTotal:
        while (now - timeBase) < (interval):
            now = time.time_ns()
        frame = camera.read()
        df.loc[len(df.index)] = [now,frame]
        timeBase += interval
        count += 1
    finish = time.time()

print('[RESULTS] waited '+ str((now - timeBase)/(10**9)) + 's')
print('[RESULTS] trigger at timestamp ' +str(now/(10**9)))
    
print('[RESULTS] captured %d frames at %.2ffps' % (
    frameTotal,
    frameTotal / (finish - start)))
df.to_csv('results.csv', index = False)
print('[RESULTS] csv exported with '+str(len(df.index))+' lines')
