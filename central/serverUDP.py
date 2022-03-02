import socket,time
import numpy as np
import warnings
warnings.filterwarnings("ignore")
from functions import processCentroids_calib
from cv2 import circle,putText,imshow,waitKey,FONT_HERSHEY_SIMPLEX,destroyAllWindows

######## YOU MAY CHANGE THIS VARIABLES ########
numberCameras,triggerTime,recTime = 1,5,2

######## PLEASE DO NOT CHANGE BELOW THIS LINE ########

#############################+-------------------------+#############################
#############################|     SENDING TRIGGER     |#############################
#############################+-------------------------+#############################
print('[INFO] creating server')
bufferSize,server_socket = 1024,socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
server_socket.bind(('0.0.0.0',8888))
count,data,verbose,capture,addDic = [0.0,0.0],[],True,np.ones(numberCameras),{}

print("[INFO] server running, waiting for clients")
for i in range(numberCameras):
    _,address = server_socket.recvfrom(bufferSize)
    addDic[address]=i
    print('[INFO] '+str(len(addDic))+' client connected')

print('[INFO] all clients connected')
myIPs = list(addDic.keys())
triggerTime += time.time()
for i in range(numberCameras):
    server_socket.sendto((str(triggerTime)+' '+str(recTime)).encode(),tuple(myIPs[i]))
print('[INFO] trigger sent, waiting for capture ...')

#############################+-------------------------+#############################
#############################|        CAPTURING        |#############################
#############################+-------------------------+#############################
try:
    while np.any(capture):
        bytesPair = server_socket.recvfrom(bufferSize)
        message = np.frombuffer(bytesPair[0])
        address = bytesPair[1]
        if not (len(message)-1): capture[addDic[address]] = 0

        if capture[addDic[address]]:
            coord,a,b,time = message[0:6].reshape(-1,2),message[6],message[7],message[8]
            undCoord = processCentroids_calib(coord,a,b)
            count[addDic[address]] += 1
            data.append([coord,a,b,undCoord])

            if verbose:
                img,k = np.ones((480,640,3))*25,0
                for pt in undCoord:
                    center = (int(np.round(pt[0]*16)), int(np.round(pt[1]*16)))
                    circle(img,center,10,(255,0,0),5,shift=4)
                    putText(img,str(k),(int(center[0]/16)-25, int(center[1]/16)-25),FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),2) 
                    k+=1
                imshow(str(address),img)
                waitKey(1)
finally:
    server_socket.close()
    destroyAllWindows()
    print('[RESULTS] server results are')
    for i in range(numberCameras):
        print('\t>> camera '+str(i)+': '+str(count[i])+' images, address '+str(myIPs[i]))
