from nis import match
import socket,time
import numpy as np
import warnings
import threading
warnings.filterwarnings("ignore")
from functions import processCentroids_calib
from cv2 import circle, compare,putText,imshow,waitKey,FONT_HERSHEY_SIMPLEX,destroyAllWindows

class myServer(object):
    def __init__(self):
        # PLEASE CHANGE JUST THE VARIABLES BELOW
        self.numberCameras,self.triggerTime,self.recTime = 2,2,30
        # DO NOT CHANGE BELOW THIS LINE
        print('[INFO] creating server')
        self.lock = threading.Lock()
        self.bufferSize,self.server_socket = 80,socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        self.server_socket.bind(('0.0.0.0',8888))
        self.data,self.verbose,self.addDic,self.myIPs,self.capture,self.FPS,self.waiting= [],False,{},(),np.ones(self.numberCameras,dtype=np.bool),40,np.zeros(self.numberCameras)
        for i in range(self.numberCameras): self.data.append([])

    def connect(self):
        print("[INFO] server running, waiting for clients")
        for i in range(self.numberCameras):
            _,address = self.server_socket.recvfrom(self.bufferSize)
            self.addDic[address]=i
            print('[INFO] client '+str(len(self.addDic))+' connected at '+str(address[0]))
        print('[INFO] all clients connected')
        self.myIPs = list(self.addDic.keys())
        self.triggerTime += time.time()
        for i in range(self.numberCameras):
            self.server_socket.sendto((str(self.triggerTime)+' '+str(self.recTime)).encode(),tuple(self.myIPs[i]))
        print('[INFO] trigger sent')
    
    def collect(self):
        print('[INFO] waiting capture')
        baseIdx = False
        try:
            while np.any(self.capture):
                bytesPair = self.server_socket.recvfrom(self.bufferSize)
                message = np.frombuffer(bytesPair[0],dtype=np.float64)
                address = bytesPair[1]
                if not (len(message)-1): self.capture[self.addDic[address]] = 0

                if self.capture[self.addDic[address]]:
                    coord,a,b,time,imgNumber,idx = message[0:6].reshape(-1,2),message[6],message[7],message[8],message[9],self.addDic[address]
                    undCoord = processCentroids_calib(coord,a,b)
                    self.data[idx].append(np.concatenate((undCoord.reshape(6),[time,imgNumber])))
                    self.data[idx]=sorted(self.data[idx], key=lambda x: x[6])     
                    
                    if len(self.data[int(not(idx))])>1 and not imgNumber and not baseIdx:
                        compareCam = int(not(idx))
                        # print(abs(time-self.data[compareCam][0][6]),abs(time-self.data[compareCam][1][6]))
                        # verify if img number is really 1 and 0
                        if abs(time-self.data[compareCam][0][6]) > abs(time-self.data[compareCam][1][6]): del self.data[compareCam][0]
                        baseIdx = True

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
            for i in range(self.numberCameras): print('  >> camera '+str(i)+': '+str(len(self.data[i]))+' valid images, address '+str(self.myIPs[i][0])+', FPS '+str(len(self.data[i])/self.recTime))
            np.savetxt("cam1.csv",self.data[0],delimiter =", ",fmt ='% s')
            np.savetxt("cam2.csv",self.data[1],delimiter =", ",fmt ='% s')
            diff = np.array(abs(np.array([row[6] for row in self.data[0]])-np.array([row[6] for row in self.data[1]])))
            print(diff.min(),np.mean(diff),diff.max())
            print((diff > 0.5/40).nonzero())


myServer_ = myServer()
tCollect = threading.Thread(target=myServer_.collect, args=[])
#tOrder = threading.Thread(target=myServer_.order, args=[])
#tOrder.start()
myServer_.connect()
tCollect.start()
tCollect.join()
#tOrder.join()