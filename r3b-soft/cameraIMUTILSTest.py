from imutils.video.pivideostream import PiVideoStream
import imutils
import time
import cv2
from multiprocessing.pool import Pool
import socket
import os
import pandas as pd
import numpy as np

os.system('clear')
os.system('rm -rf pics/*jpg')

print("[INFO] creating saving thread function")
def write2disk(i,frame):    
    print("in thread",i)
    cv2.imwrite('pics/'+str(i).zfill(4)+'.bmp',frame)
    
print("[INFO] setting global parameters")
serverAdressPort = ("192.168.1.103", 8888)
bufferSize = 1024
UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

print("[INFO] creating camera object")
camera = PiVideoStream(resolution=(640,480),framerate=90).start()
time.sleep(2.0)

print("[INFO] starting thread pool and image stream")
i = 0
pool = Pool(50)

print("[INFO] triggering server")
UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
UDPClientSocket.sendto(str.encode("on"),serverAdressPort)
msgFromServer = UDPClientSocket.recvfrom(bufferSize)
data = msgFromServer[0].split()
timeBase = int(data[0])
frames = int(data[1])
interval = int(data[2])
df = pd.DataFrame(index=np.arange(frames),
                  columns=['diff to base (sec)','timestamp(microsec)'])

print("[INFO] capturing images")
now = time.time_ns()
start = time.time()+1

while i < frames:
    # wait for timestamp
    while (now-timeBase) < interval:
        now = time.time_ns()
    # read stream
    image=camera.read()
    # call thread for saving
    pool.apply_async(write2disk,(i,image))
    # store time diff
    df.loc[i] = [(now - timeBase)/(10**9),now/(10**6)]
    # update variables
    i+=1
    timeBase+=interval
finish = time.time()
print(df)
print("[RESULTS] elasped time: ",(finish-start))
print("[RESULTS] approx. FPS:",frames/(finish - start))
pool.close()
pool.join()
camera.stop()

