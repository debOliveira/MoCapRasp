import picamera
import picamera.array
import time
import cv2
from multiprocessing.pool import Pool
from texttable import Texttable
import socket

print("[INFO] creating saving thread function")
def write2disk(frame,i):    
    print("in thread",i)
    cv2.imwrite('pics/'+str(i).zfill(4)+'.jpg',frame)
    
print("[INFO] setting global parameters")
frames = 3
interval = 15000000
t = Texttable()
t.header(('iteration number', 'diff to base (sec)', 'now timestamp (milisec)'))
t.set_cols_width([20,20,20])
t.set_cols_dtype(["t", "t", "t"])
localIp = "192.168.1.103"
localPort = 8888
bufferSize = 1024
UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
UDPServerSocket.bind((localIp,localPort))
print("[INFO] server running")

print("[INFO] creating camera object")
camera = picamera.PiCamera()
camera.resolution = (640,480)
camera.sensor_mode = 7
camera.framerate = 90
time.sleep(2.0)

print("[INFO] starting thread pool and image stream")
i = 0
pool = Pool(5)
stream = picamera.array.PiRGBArray(camera)

'''print("[INFO] waiting for client")
bytesPair = UDPServerSocket.recvfrom(bufferSize)
adress = bytesPair[1]'''
timeBase = time.time_ns()+10**9
'''print(str(adress) + ' >> '+ str(int(timeBase)/(10**9)))
UDPServerSocket.sendto(str.encode(
    str(timeBase)+' '+str(frames)+' '+str(interval)),adress)'''

print("[INFO] capturing images")
now = time.time_ns()
start = time.time()+1

while i < frames:
    while (now-timeBase) < interval:
        now = time.time_ns()
    camera.capture(stream, format='bgr')
    image = stream.array
    stream.seek(0)
    stream.truncate()
    #print(image)
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
