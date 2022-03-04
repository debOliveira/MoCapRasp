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
        self.numberCameras,self.triggerTime,self.recTime = 2,5,30
        # DO NOT CHANGE BELOW THIS LINE
        print('[INFO] creating server')
        self.lock = threading.Lock()
        self.bufferSize,self.server_socket = 80,socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        self.server_socket.bind(('0.0.0.0',8888))
        self.data,self.verbose,self.addDic,self.myIPs,self.capture,self.FPS = [],False,{},(),np.ones(self.numberCameras,dtype=np.bool),40
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
        try:
            while np.any(self.capture):
                bytesPair = self.server_socket.recvfrom(self.bufferSize)
                message = np.frombuffer(bytesPair[0],dtype=np.float64)
                address = bytesPair[1]
                if not (len(message)-1): self.capture[self.addDic[address]] = 0

                if self.capture[self.addDic[address]]:
                    coord,a,b,time,number = message[0:6].reshape(-1,2),message[6],message[7],message[8],message[9]
                    undCoord = processCentroids_calib(coord,a,b)
                    self.data[self.addDic[address]].append(np.concatenate((undCoord.reshape(6),[time])))
                    if len(self.data[self.addDic[address]]):
                        self.data[self.addDic[address]]=sorted(self.data[self.addDic[address]], key=lambda x: x[6])

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
            for i in range(self.numberCameras): print('  >> camera '+str(i)+': '+str(len(self.data[i]))+' images, address '+str(self.myIPs[i][0])+', FPS '+str(len(self.data[i])/self.recTime))
            timeDiff1,timeDiff2 =[],[]
            for i in range(len(self.data[0])):  timeDiff1.append(self.data[0][i][6])
            for i in range(len(self.data[1])):  timeDiff2.append(self.data[1][i][6])
            print('[RESULTS] FPS review')
            print('#  min  mean   max ')
            print(1,round(np.diff(timeDiff1).min(),3),round(np.mean(np.diff(timeDiff1)),3),round(np.diff(timeDiff1).max(),3))
            print(2,round(np.diff(timeDiff2).min(),3),round(np.mean(np.diff(timeDiff2)),3),round(np.diff(timeDiff2).max(),3))
            np.savetxt("cam1.csv",self.data[0],delimiter =", ",fmt ='% s')
            np.savetxt("cam2.csv",self.data[1],delimiter =", ",fmt ='% s')

    def checkMatch(self,nPrevious,baseCam,compareCam):
        compareMatch,baseTime = np.zeros(nPrevious),self.data[baseCam][-int(nPrevious/2)][6]
        for i in range(0,nPrevious): compareMatch[i]=abs(self.data[compareCam][-(i+1)][6]-baseTime)
        return compareMatch.min()

    def getBase(self):       
        notEmpty,firstTime = np.zeros(self.numberCameras,dtype=np.bool),np.zeros(self.numberCameras)
        while True:
            for i in range(self.numberCameras): 
                if len(self.data[i]): notEmpty[i],firstTime[i] = True,self.data[i][0][6]
            if np.sum(notEmpty) == self.numberCameras:
                order = np.flip(np.argsort(firstTime))
                return order
        
    def match(self):
        lastLen,nPrevious,count = 0,10,0
        baseCam,otherCam = self.getBase()
        print('[INFO] chosen base is '+str(self.myIPs[baseCam][0]))
        while np.any(self.capture):
            with self.lock:
                if (len(self.data[otherCam])>nPrevious) and (len(self.data[baseCam])>nPrevious/2) and len(self.data[baseCam])!=lastLen: 
                    if self.checkMatch(nPrevious,baseCam,otherCam)>0.5/self.FPS: 
                        count+=1
                        #print(self.checkMatch(nPrevious,baseCam,otherCam))
                    lastLen=len(self.data[baseCam])
        print('[RESULTS] discarded images via sync mismatch = '+str(count))

myServer_ = myServer()
tMatch = threading.Thread(target=myServer_.match, args=[])
tCollect = threading.Thread(target=myServer_.collect, args=[])

tMatch.start()
myServer_.connect()
tCollect.start()

tCollect.join()
tMatch.join()
