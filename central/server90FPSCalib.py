from nis import match
import socket,time
import numpy as np
import warnings
import threading
warnings.filterwarnings("ignore")
from functions import processCentroids_calib,map1_cam1,map1_cam2,map2_cam1,map2_cam2,cameraMatrix_cam1,cameraMatrix_cam2,distCoef_cam1,distCoef_cam2
from cv2 import circle,putText,imshow,waitKey,FONT_HERSHEY_SIMPLEX,destroyAllWindows,triangulatePoints,moveWindow,imwrite
from myLib import orderCenterCoord,getPreviousCentroid,estimateFundMatrix_8norm,decomposeEssentialMat,myProjectionPoints,isCollinear
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class myServer(object):
    def __init__(self):
        # PLEASE CHANGE JUST THE VARIABLES BELOW
        self.numberCameras,self.triggerTime,self.recTime = 2,5,2000
        print('[INFO] creating server')
        self.lock = threading.Lock()
        self.bufferSize,self.server_socket = 80,socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        self.server_socket.bind(('0.0.0.0',8888))
        self.data,self.verbose,self.addDic,self.myIPs,self.capture,self.FPS,self.match,self.myIPs,self.miss= [],True,{},(),np.ones(self.numberCameras,dtype=np.bool),40,[],[],0
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
                    if sizeMsg != 10:
                        print('[ERROR] '+str(int((sizeMsg-4)/2))+' were found')
                        self.data[idx].append([0,0,0,0,0,0,0,0,False])
                    else:
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
            np.savetxt("cam1.csv",[row[0:6] for row in self.data[0]],delimiter =", ",fmt ='% s')
            np.savetxt("cam2.csv",[row[0:6] for row in self.data[1]],delimiter =", ",fmt ='% s')
    
    def order(self,idxBase=0,idxCompare=1):
        hasPrevious,centroids1,centroids2,invalid = False,[0],[0],0
        while len(self.match) or np.any(self.capture):
            if len(self.match):
                imgNumber = self.match[0]
                pts1,pts2 = np.array(self.data[idxBase][imgNumber][0:6]).reshape(-1,2),np.array(self.data[idxCompare][imgNumber][0:6]).reshape(-1,2)

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

                    hasPrevious = True
                    print('matched ',imgNumber)

                    if self.verbose:
                        img,k = np.ones((480,640,3))*25,0
                        for pt in sorted1:
                            center = (int(np.round(pt[0]*16)), int(np.round(pt[1]*16)))
                            circle(img,center,10,(255,0,0),5,shift=4)
                            putText(img,str(k),(int(center[0]/16)-25, int(center[1]/16)-25),FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),2) 
                            k+=1
                        imshow(str(idxBase),img)
                        moveWindow(str(idxCompare), 10,10);
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
                        moveWindow(str(idxCompare), 10,10);
                        #imwrite(str(idxCompare)+'/'+str(k).zfill(4)+'.jpg')
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
        np.savetxt("R.csv",R,delimiter =", ",fmt ='% s')
        np.savetxt("t.csv",t,delimiter =", ",fmt ='% s')
        P1,P2 = np.hstack((cameraMatrix_cam1, [[0.], [0.], [0.]])),np.matmul(cameraMatrix_cam2, np.hstack((R, t.T)))
        projPt1,projPt2 = myProjectionPoints(np.array(centroids1)),myProjectionPoints(np.array(centroids2))

        points4d = triangulatePoints(P1.astype(float),
                                        P2.astype(float),
                                        projPt1.astype(float),
                                        projPt2.astype(float))
        points3d = (points4d[:3, :]/points4d[3, :]).T

        if points3d[0, 2] < 0: points3d = -points3d

        print("\nRot. Mat.\n", R.round(4))
        print("\nTrans. Mat.\n", t.round(4))

        tot,L_real_AC,L_real_AB,L_real_BC,L_AC_vec,L_BC_vec,L_AB_vec,k = 0,15.7,10.4,5.3,[],[],[],0
        for [A, B, C] in points3d.reshape([-1, 3, 3]):
            L_rec_AC,L_rec_BC,L_rec_AB = np.linalg.norm(A-C),np.linalg.norm(B-C),np.linalg.norm(A-B)
            if L_rec_AB<L_rec_BC: L_rec_AB,L_rec_BC=L_rec_BC,L_rec_AB
            tot = tot + L_real_AC/L_rec_AC + L_real_BC/L_rec_BC + L_real_AB/L_rec_AB
            if k:
                if abs(L_rec_AB*lamb-L_real_AB)>1 or abs(L_rec_AC*lamb-L_real_AC)>1 or abs(L_rec_BC*lamb-L_real_BC)>1:
                    continue
            k = k + 3
            lamb = tot/k
            L_AC_vec.append(L_rec_AC)
            L_BC_vec.append(L_rec_BC)
            L_AB_vec.append(L_rec_AB)


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
        plt.xlim(0,len(L_AC_vec)-1)
        plt.draw()
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

        fig = plt.figure(figsize=(8, 8))
        ax = plt.axes(projection='3d')
        ax.set_xlim(-0.1, 3.1)
        ax.set_zlim(-0.4, 0.4)
        ax.set_ylim(-0.1, 4)
        ax.set_xlabel('X')
        ax.set_ylabel('Z')
        ax.set_zlabel('Y')

        scale = 0.3
        ax.quiver(0, 0, 0, scale, 0, 0,  # x0,y0,z0,x1,y1,z1
                arrow_length_ratio=0.1, edgecolors="r")
        ax.quiver(0, 0, 0, 0, 0, scale,
                arrow_length_ratio=0.1, edgecolors="b")
        ax.quiver(0, 0, 0, 0, scale, 0,
                arrow_length_ratio=0.1, edgecolors="g")
        ax.scatter(0, 0, 0, edgecolor="blue", facecolor="black")

        x = np.array([scale, 0, 0])
        y = np.array([0, scale, 0])
        z = np.array([0, 0, scale])
        x = np.matmul(R.T, x)
        y = np.matmul(R.T, y)
        z = np.matmul(R.T, z)

        t_aux = np.matmul(-t, R)*lamb/100

        ax.quiver(t_aux[0][0], t_aux[0][2], t_aux[0][1],
                x[0], x[2], x[1], arrow_length_ratio=0.1, edgecolors="r")
        ax.quiver(t_aux[0][0], t_aux[0][2], t_aux[0][1],
                y[0], y[2], y[1], arrow_length_ratio=0.1, edgecolors="b")
        ax.quiver(t_aux[0][0], t_aux[0][2], t_aux[0][1],
                z[0], z[2], z[1], arrow_length_ratio=0.1, edgecolors="g")
        ax.scatter(t_aux[0][0], t_aux[0][2], t_aux[0][1],
                edgecolor="r", facecolor="gold")

        cmhot = plt.get_cmap("jet")

        #ax.view_init(elev=0, azim=-90)  # -37.5,30
        #ax.view_init(elev=-70, azim=-120)  # -37.5,30
        ax.view_init(elev=35, azim=-110)  # -37.5,30
        ax.scatter(points3d[:, 0]*lamb/100, points3d[:, 2]*lamb/100,
                points3d[:, 1]*lamb/100, c=points3d[:, 2], cmap=cmhot)
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