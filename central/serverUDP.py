import socket
import numpy as np
import warnings
warnings.filterwarnings("ignore")
from functions import processCentroids_test
from cv2 import circle,putText,imshow,waitKey,FONT_HERSHEY_SIMPLEX

bufferSize = 1024
server_socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
server_socket.bind(('0.0.0.0',8888))
print("[INFO] server running, waiting...")

count,data,verbose = 0,[],True

while True:
    bytesPair = server_socket.recvfrom(bufferSize)
    message = np.frombuffer(bytesPair[0])
    if not (len(message)-1): break
    coord,a,b,time = message[0:8].reshape(-1,2),message[8],message[9],message[10]
    undCoord = processCentroids_test(coord,a,b)
    count += 1
    data.append([coord,a,b,undCoord])

    if verbose:
        img,k = np.ones((480,640,3))*25,0
        for pt in undCoord:
            center = (int(np.round(pt[0]*16)), int(np.round(pt[1]*16)))
            circle(img,center,10,(255,0,0),5,shift=4)
            putText(img,str(k),(int(center[0]/16)-25, int(center[1]/16)-25),FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),2) 
            k+=1
        imshow('img',img)
        waitKey(1)

server_socket.close()
print('[RESULTS] processed '+str(count)+' images')

# multiple thread server
# order centroids (needs 2 cameras)
# triangulate