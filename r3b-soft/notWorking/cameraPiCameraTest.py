import picamera
import picamera.array
import imutils
import time
import cv2
from multiprocessing.pool import Pool
from texttable import Texttable
import socket

print("[INFO] creating saving thread function")
def write2disk(frame,i):    
    print("in thread", i)
    cv2.imwrite('pics/'+str(i).zfill(4)+'.jpg',frame)
    
print("[INFO] setting global parameters")
t = Texttable()
t.header(('iteration number', 'diff to base (sec)', 'now timestamp (milisec)'))
t.set_cols_width([20,20,20])
t.set_cols_dtype(["t", "t", "t"])
serverAdressPort = ("192.168.1.103", 8888)
bufferSize = 1024
UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

print("[INFO] creating camera object")
camera = picamera.PiCamera(resolution=(640,480))
camera.resolution = (640,480)
camera.sensor_mode = 7
camera.framerate = 90
time.sleep(2.0)

print("[INFO] starting thread pool and image stream")
i = 0
pool = Pool(5)
stream = picamera.array.PiRGBArray(camera)

print("[INFO] triggering server")
UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
UDPClientSocket.sendto(str.encode("on"),serverAdressPort)
msgFromServer = UDPClientSocket.recvfrom(bufferSize)
data = msgFromServer[0].split()
timeBase = int(data[0])
frames = int(data[1])
interval = int(data[2])

print("[INFO] capturing images")
now = time.time_ns()
start = time.time()

while i < frames:
    while (now-timeBase) < interval:
        now = time.time_ns()
    image = stream.array
    pool.apply_async(write2disk,(image,i,))
    t.add_row((i, (now - timeBase)/(10**9), now/(10**6)))
    i+=1
    timeBase+=interval
    
finish = time.time()
print(t.draw())
print("[RESULTS] elasped time: ",(finish-start))
print("[RESULTS] approx. FPS:",frames/(finish - start))
pool.close()
pool.join()
