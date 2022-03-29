from nis import match
import socket,time
import numpy as np
import warnings
import threading
warnings.filterwarnings("ignore")
from functions import processCentroids_calib,map1_cam1,map1_cam2,map2_cam1,map2_cam2,cameraMatrix_cam1,cameraMatrix_cam2,distCoef_cam1,distCoef_cam2
from cv2 import circle,putText,imshow,waitKey,FONT_HERSHEY_SIMPLEX,destroyAllWindows,triangulatePoints,moveWindow,imwrite
from myLib import orderCenterCoord,getPreviousCentroid,estimateFundMatrix_8norm,decomposeEssentialMat,myProjectionPoints,isCollinear,isEqual
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class myServer(object):
    def __init__(self):
        # PLEASE CHANGE JUST THE VARIABLES BELOW
        self.numberCameras,self.triggerTime,self.recTime = 2,5,1000
        print('[INFO] creating server')
        self.lock = threading.Lock()
        self.bufferSize,self.server_socket = 1024,socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        self.server_socket.bind(('0.0.0.0',8888))
        self.data,self.verbose,self.addDic,self.myIPs,self.capture,self.FPS,self.match,self.myIPs,self.miss= [],False,{},(),np.ones(self.numberCameras,dtype=np.bool),40,[],[],0
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
                address,sizeMsg = bytesPair[1],len(message)
                idx = self.addDic[address[0]]
                if not (sizeMsg-1): self.capture[idx] = 0

                if self.capture[idx]:
                    if sizeMsg < 13:
                        print('[ERROR] '+str(int((sizeMsg-4)/3))+' were found')
                        self.data[idx].append([0,0,0,0,0,0,0,0,False])
                    else:
                        msg = message[0:sizeMsg-4].reshape(-1,3)
                        coord,size = msg[:,0:2],msg[:,2].reshape(-1)
                        if sizeMsg > 13:
                            orderAscDiameters = np.argsort(size)
                            coord = np.array([coord[orderAscDiameters[-1]],coord[orderAscDiameters[-2]],coord[orderAscDiameters[-3]]]).reshape(-1,2)

                        a,b,time,imgNumber = message[-4],message[-3],message[-2],int(message[-1])
                        if address[0] == '192.168.0.102': undCoord = processCentroids_calib(coord,a,b,cameraMatrix_cam1,distCoef_cam1)
                        else: undCoord = processCentroids_calib(coord,a,b,cameraMatrix_cam2,distCoef_cam2)

                        for [A,B,C] in undCoord.reshape([-1, 3, 2]):
                            if np.linalg.norm(A-B)<(size[0]+size[1])/2 or np.linalg.norm(A-C)<(size[0]+size[2])/2 or np.linalg.norm(B-C)<(size[1]+size[2])/2:
                                self.data[idx].append([0,0,0,0,0,0,0,0,False])
                                print('bigg diam')
                                continue

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
        hasPrevious,centroids1,centroids2,invalid = False,[0],[0],0
        R,t = np.genfromtxt('/home/debora/Desktop/calibResults/R.csv', delimiter=','),np.genfromtxt('/home/debora/Desktop/calibResults/t.csv', delimiter=',').reshape(-1,3)
        lamb = np.genfromtxt('/home/debora/Desktop/calibResults/lamb.csv', delimiter=',')
        P1,P2 = np.hstack((cameraMatrix_cam1, [[0.], [0.], [0.]])),np.matmul(cameraMatrix_cam2, np.hstack((R, t.T)))
        while len(self.match) or np.any(self.capture):
            if len(self.match):
                imgNumber = self.match[0]
                pts1,pts2 = np.array(self.data[idxBase][imgNumber][0:6]).reshape(-1,2),np.array(self.data[idxCompare][imgNumber][0:6]).reshape(-1,2)
                if not np.all(pts1) or not np.all(pts2) or np.any(pts1<0) or np.any(pts2<0) or isEqual(pts1) or isEqual(pts2):
                    print('invalid pts ',imgNumber)
                    invalid+=1
                    del self.match[0]
                    continue
                if isCollinear(*pts1) and isCollinear(*pts2):     
                    if invalid>=10: prev1,prev2 = [],[]           
                    else: prev1,prev2 = getPreviousCentroid(hasPrevious, centroids1[len(centroids1)-3:(len(centroids1))]),getPreviousCentroid(hasPrevious, centroids2[len(centroids2)-3:(len(centroids2))])
                    sorted1, otherCamOrder = orderCenterCoord(pts1,prev1)
                    sorted2, _ = orderCenterCoord(pts2,prev2,otherCamOrder)
                    #print(otherCamOrder,sorted1,sorted2)
                    if not hasPrevious:
                        centroids1,centroids2 = np.copy(sorted1),np.copy(sorted2)
                    else:
                        centroids1,centroids2 = np.vstack((centroids1, sorted1)),np.vstack((centroids2, sorted2))

                    hasPrevious,invalid = True,0
                    print('matched ',imgNumber)

                    if self.verbose:
                        img,k = np.ones((480,640,3))*25,0
                        for pt in sorted1:
                            center = (int(np.round(pt[0]*16)), int(np.round(pt[1]*16)))
                            circle(img,center,10,(255,0,0),5,shift=4)
                            putText(img,str(k),(int(center[0]/16)-25, int(center[1]/16)-25),FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),2) 
                            k+=1
                        imshow(str(idxBase),img)
                        moveWindow(str(idxBase), 0,10);
                        waitKey(1)
                        #imwrite(str(idxBase)+'/'+str(k).zfill(4)+'.jpg')
                        img,k = np.ones((480,640,3))*25,0
                        for pt in sorted2:
                            center = (int(np.round(pt[0]*16)), int(np.round(pt[1]*16)))
                            circle(img,center,10,(255,0,0),5,shift=4)
                            putText(img,str(k),(int(center[0]/16)-25, int(center[1]/16)-25),FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),2) 
                            k+=1
                        imshow(str(idxCompare),img)
                        waitKey(1)
                        moveWindow(str(idxCompare), 700,10);
                        #imwrite(str(idxCompare)+'/'+str(k).zfill(4)+'.jpg')
                else: 
                    print('non collinear ', imgNumber)
                    invalid+=1
                del self.match[0]
        

        projPt1,projPt2 = myProjectionPoints(np.array(centroids1)),myProjectionPoints(np.array(centroids2))

        points4d = triangulatePoints(P1.astype(float),
                                        P2.astype(float),
                                        projPt1.astype(float),
                                        projPt2.astype(float))
        points3d = (points4d[:3, :]/points4d[3, :]).T

        if points3d[0, 2] < 0: points3d = -points3d

        L_real_AC,L_real_AB,L_real_BC,L_AC_vec,L_BC_vec,L_AB_vec = 15.7,10.2,5.5,[],[],[]
        for [A, B, C] in points3d.reshape([-1, 3, 3]):
            L_rec_AC = np.linalg.norm(A-C)
            L_rec_BC = np.linalg.norm(B-C)
            L_rec_AB = np.linalg.norm(A-B)
            if L_rec_AB<L_rec_BC: L_rec_AB,L_rec_BC=L_rec_BC,L_rec_AB
            L_AC_vec.append(L_rec_AC)
            L_BC_vec.append(L_rec_BC)
            L_AB_vec.append(L_rec_AB)

        print('Scale between real world and triang. point cloud is: ', lamb.round(2))
        print('L_AC >> mean = ' + str((np.mean(L_AC_vec)*lamb).round(4)) +
            "cm, std. dev = " + str((np.std(L_AC_vec)*lamb).round(4)) +
            "cm, rms = " + str((np.sqrt(np.mean(np.square(np.array(L_AC_vec)*lamb-L_real_AC)))).round(4)) + "cm")
        print('L_AB >> mean = ' + str((np.mean(L_AB_vec)*lamb).round(4)) +
            "cm, std. dev = " + str((np.std(L_AB_vec)*lamb).round(4)) +
            "cm, rms = " + str((np.sqrt(np.mean(np.square(np.array(L_AB_vec)*lamb-L_real_AB)))).round(4)) + "cm")
        print('L_BC >> mean = ' + str((np.mean(L_BC_vec)*lamb).round(4)) +
            "cm, std. dev = " + str((np.std(L_BC_vec)*lamb).round(4)) +
            "cm, rms = " + str((np.sqrt(np.mean(np.square(np.array(L_BC_vec)*lamb-L_real_BC)))).round(4)) + "cm")
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
        plt.xlim(0,len(L_AC_vec)-1)
        plt.draw()
        plt.show()

        fig = plt.figure(figsize=(8, 8))
        ax = plt.axes(projection='3d')
        ax.set_xlim(-0.1, 3.1)
        ax.set_zlim(-0.4, 0.4)
        ax.set_ylim(-0.1, 4)
        ax.set_xlabel('X')
        ax.set_ylabel('Z')
        ax.set_zlabel('Y')

        scale = 0.3
        x = np.array([scale, 0, 0])
        y = np.array([0, scale, 0])
        z = np.array([0, 0, scale])
        t_aux = np.matmul(-t, R)*lamb/100
        theta = np.arctan2(t_aux[0][1],t_aux[0][0])
        Rz = np.array([[np.cos(-theta),np.sin(-theta),0],
                    [np.sin(-theta), np.cos(-theta),0],
                    [0,0,1]])
        x = np.matmul(Rz, x)
        y = np.matmul(Rz, y)
        z = np.matmul(Rz, z)
        ax.quiver(0, 0, 0,
                x[0], x[2], x[1], arrow_length_ratio=0.1, edgecolors="r")
        ax.quiver(0, 0, 0,
                y[0], y[2], y[1], arrow_length_ratio=0.1, edgecolors="b")
        ax.quiver(0, 0, 0,
                z[0], z[2], z[1], arrow_length_ratio=0.1, edgecolors="g")
        ax.scatter(0, 0, 0, edgecolor="blue", facecolor="black")

        x = np.array([scale, 0, 0])
        y = np.array([0, scale, 0])
        z = np.array([0, 0, scale])
        x = np.matmul(Rz,np.matmul(R.T, x))
        y = np.matmul(Rz,np.matmul(R.T, y))
        z = np.matmul(Rz,np.matmul(R.T, z))

        t_new = np.matmul(Rz,t_aux.T).T
        points3d_new = np.matmul(Rz,points3d.T).T

        ax.quiver(t_new[0][0], t_new[0][2], t_new[0][1],
                x[0], x[2], x[1], arrow_length_ratio=0.1, edgecolors="r")
        ax.quiver(t_new[0][0], t_new[0][2], t_new[0][1],
                y[0], y[2], y[1], arrow_length_ratio=0.1, edgecolors="b")
        ax.quiver(t_new[0][0], t_new[0][2], t_new[0][1],
                z[0], z[2], z[1], arrow_length_ratio=0.1, edgecolors="g")
        ax.scatter(t_new[0][0], t_new[0][2], t_new[0][1],
                edgecolor="r", facecolor="gold")

        cmhot = plt.get_cmap("jet")

        #ax.view_init(elev=0, azim=-90)  # -37.5,30
        #ax.view_init(elev=-70, azim=-120)  # -37.5,30
        ax.view_init(elev=15, azim=-110)  # -37.5,30
        ax.scatter(points3d_new[:, 0]*lamb/100, points3d_new[:, 2]*lamb/100,
                points3d_new[:, 1]*lamb/100, c=points3d_new[:, 2], cmap=cmhot)
        plt.gca().invert_zaxis()
        ax.get_proj = lambda: np.dot(Axes3D.get_proj(ax), np.diag([1., 1., .3, 1.]))
        plt.draw()
        plt.show()


myServer_ = myServer()
tCollect = threading.Thread(target=myServer_.collect, args=[])
tOrder = threading.Thread(target=myServer_.order, args=[])
tOrder.start()
myServer_.connect()
tCollect.start()
tCollect.join()
tOrder.join()