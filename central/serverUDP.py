import socket,time
import numpy as np
import warnings
import threading
warnings.filterwarnings("ignore")
from functions import processCentroids_calib
from cv2 import circle,putText,imshow,waitKey,FONT_HERSHEY_SIMPLEX,destroyAllWindows

class myServer(object):
    def __init__(self):
        # PLEASE CHANGE JUST THE VARIABLES BELOW
        self.numberCameras,self.triggerTime,self.recTime = 2,5,2
        # DO NOT CHANGE BELOW THIS LINE
        print('[INFO] creating server')
        self.lock = threading.Lock()
        self.bufferSize,self.server_socket = 1024,socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        self.server_socket.bind(('0.0.0.0',8888))
        self.data,self.verbose,self.addDic,self.myIPs = [],True,{},()
        for i in range(self.numberCameras): self.data.append([])

    def connect(self):
        print("[INFO] server running, waiting for clients")
        for i in range(self.numberCameras):
            _,address = self.server_socket.recvfrom(self.bufferSize)
            self.addDic[address]=i
            print('[INFO] '+str(len(self.addDic))+' client connected')
        print('[INFO] all clients connected')
        self.myIPs = list(self.addDic.keys())
        self.triggerTime += time.time()
        for i in range(self.numberCameras):
            self.server_socket.sendto((str(self.triggerTime)+' '+str(self.recTime)).encode(),tuple(self.myIPs[i]))
        print('[INFO] trigger sent')
    
    def run(self):
        print('[INFO] waiting capture')
        capture = np.ones(self.numberCameras)
        try:
            while np.any(capture):
                bytesPair = self.server_socket.recvfrom(self.bufferSize)
                message = np.frombuffer(bytesPair[0])
                address = bytesPair[1]
                if not (len(message)-1): capture[self.addDic[address]] = 0

                if capture[self.addDic[address]]:
                    coord,a,b,time = message[0:6].reshape(-1,2),message[6],message[7],message[8]
                    undCoord = processCentroids_calib(coord,a,b)
                    self.data[self.addDic[address]].append([undCoord,time])

                    if self.verbose:
                        img,k = np.ones((480,640,3))*25,0
                        for pt in undCoord:
                            center = (int(np.round(pt[0]*16)), int(np.round(pt[1]*16)))
                            circle(img,center,10,(255,0,0),5,shift=4)
                            putText(img,str(k),(int(center[0]/16)-25, int(center[1]/16)-25),FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),2) 
                            k+=1
                        imshow(str(address),img)
                        waitKey(1)
        finally:
            self.server_socket.close()
            destroyAllWindows()
            print('[RESULTS] server results are')
            for i in range(self.numberCameras):
                print('          >> camera '+str(i)+': '+str(len(self.data[i]))+' images, address '+str(self.myIPs[i]))


myServer_ = myServer()
myServer_.connect()
t1 = threading.Thread(myServer_.run())
t1.start()
t1.join()
