from dataclasses import dataclass
from nis import match
import socket,time
from tabnanny import verbose
import numpy as np
import warnings
import threading
warnings.filterwarnings("ignore")
from functions import processCentroids_calib,map1_cam1,map1_cam2,map2_cam1,map2_cam2,cameraMatrix_cam1,cameraMatrix_cam2,distCoef_cam1,distCoef_cam2
from cv2 import circle,putText,imshow,waitKey,FONT_HERSHEY_SIMPLEX,destroyAllWindows,triangulatePoints,moveWindow,imwrite
from myLib import orderCenterCoord,getPreviousCentroid,estimateFundMatrix_8norm,decomposeEssentialMat,myProjectionPoints,isCollinear,isEqual,swapElements,getSignal
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import CubicSpline
import math
import time as timeLib

class myServer(object):
    def __init__(self):
        # PLEASE CHANGE JUST THE VARIABLES BELOW
        self.numberCameras,self.triggerTime,self.recTime,self.step = 2,5,30,0.01
        # do not change below this line, socket variables
        self.nImages = int(self.recTime/self.step)
        print('[INFO] creating server')
        self.lock = threading.Lock()
        self.bufferSize,self.server_socket = 1024,socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        self.server_socket.bind(('0.0.0.0',8888))
        # internal variables
        self.data,self.verbose,self.addDic,self.capture,self.myIPs,self.counter= [],False,{},np.ones(self.numberCameras,dtype=np.bool),[],0
        for i in range(self.numberCameras): self.data.append([])
        self.df = np.zeros((int(self.recTime/self.step),self.numberCameras*6+1))
        self.df[:,-1] = np.linspace(0,self.recTime,self.nImages)

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

    def myInterpol(self,t,x,tNew):
        ff = CubicSpline(t, x,axis=0)
        return ff(tNew*self.step)

    def collect(self):
        print('[INFO] waiting capture')
        invalid,coRout = np.zeros(self.numberCameras,dtype=np.uint8),self.myProcessing()
        coRout.__next__()
        try:
            counter,lastTime,lastInterp = np.zeros(self.numberCameras,dtype=np.uint16),np.zeros(self.numberCameras),np.zeros(self.numberCameras)
            while np.any(self.capture):
                bytesPair = self.server_socket.recvfrom(self.bufferSize)
                message = np.frombuffer(bytesPair[0],dtype=np.float64)
                address,sizeMsg = bytesPair[1],len(message)
                idx = self.addDic[address[0]]
                if not (sizeMsg-1): self.capture[idx] = 0

                if self.capture[idx]:
                    if sizeMsg < 13: # if less than 3 blobs, discard
                        print('[ERROR] '+str(int((sizeMsg-4)/3))+' markers were found')
                        invalid[idx]+=1
                    else: 
                        msg = message[0:sizeMsg-4].reshape(-1,3)
                        coord,size = msg[:,0:2],msg[:,2].reshape(-1)
                        # if more than 3 blobs are found, get the four bigger
                        if sizeMsg > 13: 
                            orderAscDiameters = np.argsort(size)
                            coord = np.array([coord[orderAscDiameters[-1]],coord[orderAscDiameters[-2]],coord[orderAscDiameters[-3]]]).reshape(-1,2)
                        # store message parameters
                        a,b,time,imgNumber = message[-4],message[-3],message[-2],int(message[-1]) 
                        # undistort points
                        if address[0] == '192.168.0.103': 
                            undCoord = processCentroids_calib(coord,a,b,cameraMatrix_cam1,distCoef_cam1)
                        else: 
                            undCoord = processCentroids_calib(coord,a,b,cameraMatrix_cam2,distCoef_cam2)
                        # check if there is an obstruction between the blobs
                        for [A,B,C] in undCoord.reshape([-1, 3, 2]):
                            if np.linalg.norm(A-B)<(size[0]+size[1])/2 or np.linalg.norm(A-C)<(size[0]+size[2])/2 or np.linalg.norm(B-C)<(size[1]+size[2])/2: 
                                print('occlusion')
                                invalid[idx]+=1
                                continue
                        
                        # if ts if not read corectly
                        if counter[idx]:
                            if time-lastTime[idx]>1e7: 
                                print('time missmatch')
                                invalid[idx]+=1
                                continue
                        # order markers
                        if isCollinear(*undCoord) and not isEqual(undCoord) or np.any(undCoord<0):     
                            if invalid[idx]>=10 or not counter[idx]: prev = []        
                            else: prev = np.array(self.data[idx][-1][0:6]).reshape(1,-2)
                            undCoord, _ = orderCenterCoord(undCoord,prev)
                            undCoord = np.array(undCoord)
                        else: 
                            print('not collinear')
                            invalid[idx]+=1
                            continue
                        # should this be here?
                        self.data[idx].append(np.concatenate((undCoord.reshape(6),[time,imgNumber])))
                        counter[idx]+=1
                        lastTime[idx]=time
                        # interpolate
                        if not counter[idx]%10: 
                            if counter[idx]>10: pts = np.array(self.data[idx][-11:])  
                            else: pts = np.array(self.data[idx][-10:])                        
                            coord,time = pts[:,0:6],pts[:,6]/1e6
                            lowBound,highBound = math.ceil(time[0]/self.step),math.floor(time[-1]/self.step)
                            tNew = np.linspace(lowBound,highBound,int((highBound-lowBound))+1,dtype=np.uint16)
                            self.df[tNew,int(idx*6):int(idx*6+6)] = self.myInterpol(time,coord,tNew)
                            lastInterp[idx] = tNew[-1]
                            if np.min(lastInterp)>self.counter: coRout.send(np.min(lastInterp))

        finally:
            # last interp
            for idx in range(0,self.numberCameras):
                pts = np.array(self.data[idx][-11:])  
                coord,time = pts[:,0:6],pts[:,6]/1e6
                lowBound,highBound = math.ceil(time[0]/self.step),math.floor(time[-1]/self.step)
                tNew = np.linspace(lowBound,highBound,int((highBound-lowBound))+1,dtype=np.uint16)
                self.df[tNew,int(idx*6):int(idx*6+6)] = self.myInterpol(time,coord,tNew)
                lastInterp[idx] = tNew[-1]
                if np.min(lastInterp)>self.counter: coRout.send(np.min(lastInterp))
            # close everything
            coRout.send(0)
            self.server_socket.close()
            destroyAllWindows()
            print('[RESULTS] server results are')
            for i in range(self.numberCameras): print('  >> camera '+str(i)+': '+str(len(self.data[i]))+' captured valid images images, address '+str(self.myIPs[i][0])+', missed '+str(int(invalid[i]))+' images')
            for i in range(self.numberCameras):       
                _,axs = plt.subplots(3, 2,figsize=(10,20),dpi=100)
                axs[0,0].plot(np.array(self.data[i])[:,6]/1e6, np.array(self.data[i])[:,0], 'o',label='40 FPS') 
                axs[0,0].plot(self.df[:,-1], self.df[:,i*6], '-',label='100 FPS interpolation')
                axs[0,0].set_ylabel('X - marker #0 (px)')
                axs[0,0].grid()

                axs[0,1].plot(np.array(self.data[i])[:,6]/1e6, np.array(self.data[i])[:,1], 'o',label='40 FPS') 
                axs[0,1].plot( self.df[:,-1], self.df[:,i*6+1], '-',label='100 FPS interpolation')
                axs[0,1].set_ylabel('Y - marker #0 (px)')
                axs[0,1].grid()

                axs[1,0].plot(np.array(self.data[i])[:,6]/1e6, np.array(self.data[i])[:,2], 'o',label='40 FPS') 
                axs[1,0].plot( self.df[:,-1], self.df[:,i*6+2], '-',label='100 FPS interpolation')
                axs[1,0].set_ylabel('X - marker #1 (px)')
                axs[1,0].grid()

                axs[1,1].plot(np.array(self.data[i])[:,6]/1e6, np.array(self.data[i])[:,3], 'o',label='40 FPS') 
                axs[1,1].plot(self.df[:,-1], self.df[:,i*6+3], '-',label='100 FPS interpolation')
                axs[1,1].set_ylabel('X - marker #1 (px)')
                axs[1,1].grid()

                axs[2,0].plot(np.array(self.data[i])[:,6]/1e6, np.array(self.data[i])[:,4], 'o',label='40 FPS') 
                axs[2,0].plot(  self.df[:,-1], self.df[:,i*6+4], '-',label='100 FPS interpolation')
                axs[2,0].set_xlabel('t (seconds)')
                axs[2,0].set_ylabel('X - marker #2 (px)')
                axs[2,0].grid()

                axs[2,1].plot(np.array(self.data[i])[:,6]/1e6, np.array(self.data[i])[:,5], 'o',label='40 FPS') 
                axs[2,1].plot(  self.df[:,-1], self.df[:,i*6+5], '-',label='100 FPS interpolation')
                axs[2,1].set_xlabel('t (seconds)')
                axs[2,1].set_ylabel('Y - marker #2 (px)')
                axs[2,1].grid()
                plt.draw()
                plt.show()

    def myReshaping(self,coord,tol=15):
        centerX, centerY = np.vstack((coord[:,0], coord[:,2], coord[:,4])).T,np.vstack((coord[:,1], coord[:,3], coord[:,5])).T
        distY = np.array(centerY).max(axis=1) - np.array(centerY).min(axis=1)
        if np.any(distY<tol): maxVec,minVec = centerX[:,2],centerX[:,0]
        else: maxVec,minVec = centerY[:,2],centerY[:,0]
        # get signal between markers extremities
        firstCam,secCam = getSignal(maxVec[0],minVec[0]),getSignal(maxVec[1],minVec[1])
        if firstCam != secCam: return True
        return False
        
    def myProcessing(self):        
        idxBase,idxCompare,hasPrevious,centroids1,centroids2=0,1,False,[0],[0]
        while True:
            try: 
                tNew = (yield)
                if not tNew:
                    print('end')
                    print("\n")
                    F,_ = estimateFundMatrix_8norm(np.array(centroids1),np.array(centroids2))
                    E = np.matmul(cameraMatrix_cam2.T, np.matmul(F, cameraMatrix_cam1))
                    print("\nEssenc. Mat.\n", E.round(4))

                    R, t = decomposeEssentialMat(E, cameraMatrix_cam1, cameraMatrix_cam2, np.array(centroids1), np.array(centroids2))
                    if np.any(np.isnan(R)): print('no valid rotation matrix')
                    else:
                        print("\nRot. Mat.\n", R.round(4))
                        print("\nTrans. Mat.\n", t.round(4))
                        P1,P2 = np.hstack((cameraMatrix_cam1, [[0.], [0.], [0.]])),np.matmul(cameraMatrix_cam2, np.hstack((R, t.T)))
                        projPt1,projPt2 = myProjectionPoints(np.array(centroids1)),myProjectionPoints(np.array(centroids2))
                        points4d = triangulatePoints(P1.astype(float),P2.astype(float),projPt1.astype(float),projPt2.astype(float))
                        points3d = (points4d[:3, :]/points4d[3, :]).T
                        if points3d[0, 2] < 0: points3d = -points3d
                        tot,L_real_AC,L_real_AB,L_real_BC,L_AC_vec,L_BC_vec,L_AB_vec,k = 0,15.7,10.2,5.5,[],[],[],0
                        for [A, B, C] in points3d.reshape([-1, 3, 3]):
                            L_rec_AC = np.linalg.norm(A-C)
                            L_rec_BC = np.linalg.norm(B-C)
                            L_rec_AB = np.linalg.norm(A-B)
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
                        plt.xlabel("Image number")
                        plt.ylabel("Deviation to mean value (cm)")
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
                else:
                    if self.counter: # first timestamp does not come (img on 00000 is full black)
                        print(self.counter,tNew)
                        pts1,pts2 = np.copy(self.df[int(self.counter)+1:int(tNew)+1,int(idxBase*6):int(idxBase*6+6)]),np.copy(self.df[int(self.counter)+1:int(tNew)+1,int(idxCompare*6):int(idxCompare*6+6)])
                        for i in range(int(tNew-self.counter)):

                            swap = self.myReshaping(np.vstack((pts1[i],pts2[i])))
                            if swap: 
                                aux = np.copy(pts1[i][0:2])
                                pts1[i][0:2] = np.copy(pts1[i][4:6])
                                pts1[i][4:6] = aux
                            if not hasPrevious: centroids1,centroids2 = np.copy(pts1[i].reshape(-1,2)),np.copy(pts2[i].reshape(-1,2))
                            else: centroids1,centroids2 = np.vstack((centroids1, pts1[i].reshape(-1,2))),np.vstack((centroids2, pts2[i].reshape(-1,2)))

                            ### VERBOSE ###
                            if verbose:
                                img,k = np.ones((720,960,3))*25,0
                                for pt in pts1[i].reshape(-1,2):
                                    center = (int(np.round(pt[0]*16)), int(np.round(pt[1]*16)))
                                    circle(img,center,10,(255,0,0),5,shift=4)
                                    putText(img,str(k),(int(center[0]/16)-25, int(center[1]/16)-25),FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),2) 
                                    k+=1
                                imshow(str(idxBase),img)
                                moveWindow(str(idxBase), 0,350)
                                waitKey(1)
                                #imwrite(str(idxBase)+'/'+str(k).zfill(4)+'.jpg')
                                img,k = np.ones((720,960,3))*25,0
                                for pt in pts2[i].reshape(-1,2):
                                    center = (int(np.round(pt[0]*16)), int(np.round(pt[1]*16)))
                                    circle(img,center,10,(255,0,0),5,shift=4)
                                    putText(img,str(k),(int(center[0]/16)-25, int(center[1]/16)-25),FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),2) 
                                    k+=1
                                imshow(str(idxCompare),img)
                                waitKey(1)
                                moveWindow(str(idxCompare), 700,350)
                            ### VERBOSE ###
                            hasPrevious = True
                    # first time, just update and consider the first valid counter
                    self.counter=np.copy(tNew)
            except GeneratorExit: return
            #except: continue



    def order(self,idxBase=0,idxCompare=1):
        hasPrevious,centroids1,centroids2,invalid,counter = False,[0],[0],0,0
        #R,t = np.genfromtxt('/home/debora/Desktop/calibResults/R.csv', delimiter=','),np.genfromtxt('/home/debora/Desktop/calibResults/t.csv', delimiter=',').reshape(-1,3)
        #lamb = np.genfromtxt('/home/debora/Desktop/calibResults/lamb.csv', delimiter=',')
        #P1,P2 = np.hstack((cameraMatrix_cam1, [[0.], [0.], [0.]])),np.matmul(cameraMatrix_cam2, np.hstack((R, t.T)))
        while counter<=self.nImages:
            if counter<=self.match:
                #print(counter)
                counter+=1
                #pts1,pts2 = pts[1:7],pts[7:13]
                '''if not np.all(pts1) or not np.all(pts2) or np.any(pts1<0) or np.any(pts2<0) or isEqual(pts1) or isEqual(pts2):
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
                    invalid+=1'''
        

        '''projPt1,projPt2 = myProjectionPoints(np.array(centroids1)),myProjectionPoints(np.array(centroids2))

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
        plt.show()'''


myServer_ = myServer()
myServer_.connect()
myServer_.collect()