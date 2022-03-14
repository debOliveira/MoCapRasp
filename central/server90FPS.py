from nis import match
import socket,time
import numpy as np
import warnings
import threading
warnings.filterwarnings("ignore")
from functions import processCentroids_calib,map1_cam1,map1_cam2,map2_cam1,map2_cam2,cameraMatrix_cam1,cameraMatrix_cam2,distCoef_cam1,distCoef_cam2
from cv2 import circle,putText,imshow,waitKey,FONT_HERSHEY_SIMPLEX,destroyAllWindows,triangulatePoints
from myLib import orderCenterCoord,getPreviousCentroid,estimateFundMatrix_8norm,decomposeEssentialMat,myProjectionPoints,isCollinear
import matplotlib.pyplot as plt

class myServer(object):
    def __init__(self):
        # PLEASE CHANGE JUST THE VARIABLES BELOW
        self.numberCameras,self.triggerTime,self.recTime = 2,5,1000
        print('[INFO] creating server')
        self.lock = threading.Lock()
        self.bufferSize,self.server_socket = 80,socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        self.server_socket.bind(('0.0.0.0',8888))
        self.data,self.verbose,self.addDic,self.myIPs,self.capture,self.FPS,self.match,self.myIPs,self.miss= [],False,{},(),np.ones(self.numberCameras,dtype=np.bool),70,[],[],0
        for i in range(self.numberCameras): self.data.append([])

    def connect(self):
        print("[INFO] server running, waiting for clients")
        for i in range(self.numberCameras):
            _,address = self.server_socket.recvfrom(self.bufferSize)
            self.addDic[address[0]]=i
            self.myIPs.append(address)
            print('[INFO] client '+str(len(self.addDic))+' connected at '+str(address[0]))
        print('[INFO] all clients connected')
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
                idx = self.addDic[address[0]]
                if not (len(message)-1): self.capture[idx] = 0

                if self.capture[idx]:
                    coord,a,b,time,imgNumber = message[0:6].reshape(-1,2),message[6],message[7],message[8],int(message[9])
                    if address[0] == '192.168.0.102': undCoord = processCentroids_calib(coord,a,b,cameraMatrix_cam1,distCoef_cam1)
                    else: undCoord = processCentroids_calib(coord,a,b,cameraMatrix_cam2,distCoef_cam2)
                                        
                    if len(self.data[idx])>imgNumber: self.data[idx][imgNumber] = np.concatenate((undCoord.reshape(6),[time,imgNumber,True]))
                    elif len(self.data[idx])==imgNumber: self.data[idx].append(np.concatenate((undCoord.reshape(6),[time,imgNumber,True])))
                    else:
                        for i in range(imgNumber-len(self.data[idx])): self.data[idx].append([0,0,0,0,0,0,0,0,False])
                        self.data[idx].append(np.concatenate((undCoord.reshape(6),[time,imgNumber,True])))
                    
                    idxCompare = int(not(idx))
                    if len(self.data[idxCompare])>=imgNumber+1: 
                        if self.data[idx][imgNumber][8] and self.data[idxCompare][imgNumber][8]: self.match.append(imgNumber)                                
                        else: self.miss+=1
        finally:
            self.server_socket.close()
            destroyAllWindows()
            print('[RESULTS] server results are')
            for i in range(self.numberCameras): 
                valid = [row[8] for row in self.data[i]]
                print('  >> camera '+str(i)+': '+str(int(np.sum(valid)))+' captured valid images images, address '+str(self.myIPs[i][0]))
            print('[RESULTS] missed '+str(int(self.miss))+' images')
    
    def order(self,idxBase=0,idxCompare=1):
        hasPrevious,centroids1,centroids2 = False,[0],[0]
        while len(self.match) or np.any(self.capture):
            if len(self.match):
                imgNumber = self.match[0]
                pts1,pts2 = np.array(self.data[idxBase][imgNumber][0:6]).reshape(-1,2),np.array(self.data[idxCompare][imgNumber][0:6]).reshape(-1,2)

                if self.verbose:
                    img,k = np.ones((480,640,3))*25,0
                    for pt in pts1:
                        center = (int(np.round(pt[0]*16)), int(np.round(pt[1]*16)))
                        circle(img,center,10,(255,0,0),5,shift=4)
                        putText(img,str(k),(int(center[0]/16)-25, int(center[1]/16)-25),FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),2) 
                        k+=1
                    imshow(str(idxBase),img)
                    waitKey(1)

                    img,k = np.ones((480,640,3))*25,0
                    for pt in pts2:
                        center = (int(np.round(pt[0]*16)), int(np.round(pt[1]*16)))
                        circle(img,center,10,(255,0,0),5,shift=4)
                        putText(img,str(k),(int(center[0]/16)-25, int(center[1]/16)-25),FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),2) 
                        k+=1
                    imshow(str(idxCompare),img)
                    waitKey(1)

                if isCollinear(*pts1) and isCollinear(*pts2):                
                    prev1,prev2 = getPreviousCentroid(hasPrevious, centroids1[len(centroids1)-3:(len(centroids1))]),getPreviousCentroid(hasPrevious, centroids2[len(centroids2)-3:(len(centroids2))])
                    sorted1, otherCamOrder = orderCenterCoord(pts1,prev1)
                    sorted2, _ = orderCenterCoord(pts2, prev2,otherCamOrder)
                    if not hasPrevious:
                        centroids1,centroids2 = np.copy(sorted1),np.copy(sorted2)
                    else:
                        centroids1,centroids2 = np.vstack((centroids1, sorted1)),np.vstack((centroids2, sorted2))

                    hasPrevious = True
                    print('matched ',imgNumber)
                else: print('non collinear ', imgNumber)
                del self.match[0]
        

        np.savetxt("centroids1.csv",centroids1,delimiter =", ",fmt ='% s')
        np.savetxt("centroids2.csv",centroids2,delimiter =", ",fmt ='% s')

        print("\n")
        F,_ = estimateFundMatrix_8norm(np.array(centroids1),np.array(centroids2))
        E = np.matmul(cameraMatrix_cam2.T, np.matmul(F, cameraMatrix_cam1))
        print("\nEssenc. Mat.\n", E.round(4))

        R, t = decomposeEssentialMat(E, cameraMatrix_cam1, cameraMatrix_cam2, np.array(centroids1), np.array(centroids2))
        if np.any(np.isnan(R)): return
        P1 = np.hstack((cameraMatrix_cam1, [[0.], [0.], [0.]]))
        P2 = np.matmul(cameraMatrix_cam2, np.hstack((R, t.T)))

        projPt1 = myProjectionPoints(np.array(centroids1))
        projPt2 = myProjectionPoints(np.array(centroids2))

        points4d = triangulatePoints(P1.astype(float),
                                        P2.astype(float),
                                        projPt1.astype(float),
                                        projPt2.astype(float))
        points3d = (points4d[:3, :]/points4d[3, :]).T

        if points3d[0, 2] < 0:
            points3d = -points3d

        print("\nRot. Mat.\n", R.round(4))
        print("\nTrans. Mat.\n", t.round(4))

        tot = 0
        L_real_AC = 15.7
        L_real_AB = 10.3
        L_real_BC = 5.4
        L_AC_vec = []
        L_BC_vec = []
        L_AB_vec = []
        k = 0

        for [A, B, C] in points3d.reshape([-1, 3, 3]):
            L_rec_AC = np.linalg.norm(A-C)
            L_rec_BC = np.linalg.norm(B-C)
            L_rec_AB = np.linalg.norm(A-B)
            L_AC_vec.append(L_rec_AC)
            L_BC_vec.append(L_rec_BC)
            L_AB_vec.append(L_rec_AB)
            tot = tot + L_real_AC/L_rec_AC + L_real_BC/L_rec_BC + L_real_AB/L_rec_AB
            k = k + 1

        N = points3d.shape[0]
        lamb = tot/N

        print('Scale between real world and triang. point cloud is: ', lamb.round(2))
        print('L_AC >> mean = ' + str((np.mean(L_AC_vec)*lamb).round(4)) +
            "cm, std. dev = " + str((np.std(L_AC_vec)*lamb).round(4)) + "cm")
        print('L_AB >> mean = ' + str((np.mean(L_AB_vec)*lamb).round(4)) +
            "cm, std. dev = " + str((np.std(L_AB_vec)*lamb).round(4)) + "cm")
        print('L_BC >> mean = ' + str((np.mean(L_BC_vec)*lamb).round(4)) +
            "cm, std. dev = " + str((np.std(L_BC_vec)*lamb).round(4)) + "cm")
        fig = plt.figure(figsize=(10, 6), dpi=100)
        L_AC_vec_plot = np.array(L_AC_vec)*lamb - L_real_AC
        L_BC_vec_plot = np.array(L_BC_vec)*lamb - L_real_BC
        L_AB_vec_plot = np.array(L_AB_vec)*lamb - L_real_AB
        plt.plot(L_AC_vec_plot, '-o', label="std_AC")
        plt.plot(L_BC_vec_plot, '-o', label="std_BC")
        plt.plot(L_AB_vec_plot, '-o', label="std_AB")
        plt.axhline(y=0.0, color='r', linestyle='-')
        plt.grid()
        #plt.axvline(x=853, c='r', linestyle='--', label="image 960")
        plt.xlabel("# of 3 point marker set considering all valid")
        plt.ylabel("Std. dev. (cm)")
        plt.legend()
        ax = fig.axes
        ax[0].minorticks_on()
        plt.grid(which='both')
        plt.xlim(0,len(np.array(centroids1))/3-1)
        plt.show()

        points3d_new = points3d*lamb
        i = 0
        for [A, B, C] in points3d_new.reshape([-1, 3, 3]):
            L_reconst = np.sqrt(np.sum((A-C)**2, axis=0))
            valid = abs(L_real_AC-L_reconst)/L_real_AC < 0.01
            if not valid:
                i = i + 1
        print("Images distant more than 1% from the real value = " +
            str(i)+'/'+str(int(points3d.shape[0]/3)))

myServer_ = myServer()
tCollect = threading.Thread(target=myServer_.collect, args=[])
tOrder = threading.Thread(target=myServer_.order, args=[])
tOrder.start()
myServer_.connect()
tCollect.start()
tCollect.join()
tOrder.join()