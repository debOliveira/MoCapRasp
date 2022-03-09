from nis import match
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
        self.numberCameras,self.triggerTime,self.recTime = 2,1,120
        # DO NOT CHANGE BELOW THIS LINE
        print('[INFO] creating server')
        self.lock = threading.Lock()
        self.bufferSize,self.server_socket = 80,socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        self.server_socket.bind(('0.0.0.0',8888))
        self.data,self.verbose,self.addDic,self.myIPs,self.capture,self.FPS,self.match,self.i= [],False,{},(),np.ones(self.numberCameras,dtype=np.bool),40,[],0
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
                    coord,a,b,time,imgNumber,idx = message[0:6].reshape(-1,2),message[6],message[7],message[8],int(message[9]),self.addDic[address]
                    undCoord = processCentroids_calib(coord,a,b)
                    self.data[idx].append(np.concatenate((undCoord.reshape(6),[time,imgNumber])))
                    self.data[idx]=sorted(self.data[idx], key=lambda x: x[6])  

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
            np.savetxt("cam1.csv",[row[6] for row in self.data[0]],delimiter =", ",fmt ='% s')
            np.savetxt("cam2.csv",[row[6] for row in self.data[1]],delimiter =", ",fmt ='% s')
            np.savetxt("match.csv",np.array(self.match),delimiter =", ",fmt ='% s')
    
    def order(self,activeCameras=[0,1]):
        idxBase,idxCompare = activeCameras
        while not len(self.data[idxBase]) or not len(self.data[idxCompare]): pass
        while not self.data[idxBase][0][7]==0 and self.data[idxCompare][0][7]==0: pass
        if self.data[idxBase][0][6]<self.data[idxCompare][0][6]: idxBase,idxCompare=idxCompare,idxBase

        print('[WARNING] camera priority is ['+str(idxBase)+','+str(idxCompare)+']')

        timeBase,base,missmatch = self.data[idxBase][0][6],0,0
        while True:
            while len(self.data[idxCompare])<=base: pass
            while not self.data[idxCompare][base][7]==base: pass
            if abs(timeBase-self.data[idxCompare][base][6])>0.5/self.FPS: base+=1
            else: break

        print('[WARNING] base found to '+str(base)+' >> '+str(abs(timeBase-self.data[idxCompare][base][6]))+'s difference')


        while np.any(self.capture): # review this condition
            while len(self.data[idxBase])<=(self.i) or len(self.data[idxCompare])<=(self.i+base): 
                if not np.any(self.capture): break
            else:
                # esperar tamanho do compare >= i +base e tamado da base >=i
                while not self.data[idxBase][self.i][7]==self.i and not self.data[idxCompare][self.i+base][7]==self.i+base: pass
                # confirmar numero (se não tá faltando alguma imagem antes)
                # se estiver, esperar

                timeBase=self.data[idxBase][self.i][6]
                if missmatch>=3:
                    timeCompare = self.data[idxCompare][self.i+base][6]
                    #print(self.data[idxBase][self.i][7],self.data[idxCompare][self.i+base][7],abs(timeBase-timeCompare),abs(timeBase-timeCompare)<0.5/self.FPS,base)
                    while len(self.data[idxCompare])<=(self.i+base+1): pass
                    while not self.data[idxCompare][self.i+base+1][7]==(self.i+base+1): pass
                    # se é mismatch, comparar com depois e antes (verificar se não é o primeiro elemento)
                    timeCompareMore,timeCompareLess = self.data[idxCompare][self.i+base+1][6],self.data[idxCompare][self.i+base-1][6]
                    if abs(timeBase-timeCompareMore)<abs(timeBase-timeCompare): 
                        base+=1
                        #print('add base')
                    elif abs(timeBase-timeCompareLess)<abs(timeBase-timeCompare): 
                        base-=1                      
                        #print('sub base')
                    print('[WARNING] base found to '+str(base)+' >> '+str(abs(timeBase-self.data[idxCompare][self.i+base][6]))+'s difference at '+str(self.i))

                timeCompare = self.data[idxCompare][self.i+base][6]
                # comparar time em i (na base) com i+base(no compare)
                if abs(timeBase-timeCompare)<0.5/self.FPS: 
                    self.match.append([self.i,self.i+base])
                    missmatch = 0
                else: 
                    missmatch+=1
                    #print(self.data[idxBase][self.i][7],self.data[idxCompare][self.i+base][7],abs(timeBase-timeCompare),abs(timeBase-timeCompare)<0.5/self.FPS,base)
                # se é match, manter a base e add no vetor de matchh)

                # add em i
                if self.i%self.FPS==0: print('alive check, '+str(self.i/self.FPS)+'s')
                self.i+=1
        print('[RESULTS] matched =',len(self.match),' images >> '+str(len(self.match)/min(len(self.data[idxBase]),len(self.data[idxCompare]))*100)+'%')

        for k in range(self.i,min(len(self.data[idxBase]),len(self.data[idxCompare]))-1):
            # redo loop for the images that were not processed
            print('missed ',k)




myServer_ = myServer()
tCollect = threading.Thread(target=myServer_.collect, args=[])
tOrder = threading.Thread(target=myServer_.order, args=[])
tOrder.start()
myServer_.connect()
tCollect.start()
tCollect.join()
tOrder.join()