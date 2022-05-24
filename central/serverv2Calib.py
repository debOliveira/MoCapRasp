# IMPORTS >>> DO NOT CHANGE <<<
import warnings
warnings.filterwarnings("ignore")
import socket,time,math,argparse
import numpy as np
from functions import processCentroids_calib,cameraMatrix_cam1,cameraMatrix_cam2,distCoef_cam1,distCoef_cam2
from cv2 import circle,destroyAllWindows,triangulatePoints,cvtColor,line,COLOR_GRAY2BGR,computeCorrespondEpilines
from myLib import orderCenterCoord,estimateFundMatrix_8norm,decomposeEssentialMat,myProjectionPoints,isCollinear,isEqual,getSignal
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import CubicSpline

class myServer(object):
    def __init__(self,numberCameras,triggerTime,recTime,FPS,verbose,output,save):
        ##########################################
        # PLEASE CHANGE JUST THE VARIABLES BELOW #
        ##########################################
        self.numberCameras,self.triggerTime,self.recTime,self.step,self.out,self.save = numberCameras,triggerTime,recTime,1/FPS,output,save
        self.cameraMat = np.array([cameraMatrix_cam1,cameraMatrix_cam2])
        # do not change below this line, socket variables
        self.nImages,self.imageSize = int(self.recTime/self.step),[]
        print('[INFO] creating server')
        self.bufferSize,self.server_socket = 1024,socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        self.server_socket.bind(('0.0.0.0',8888))
        # internal variables
        self.data,self.verbose,self.addDic,self.capture,self.myIPs= [],verbose,{},np.ones(self.numberCameras,dtype=np.bool),[]
        for i in range(self.numberCameras): self.data.append([])
        self.invalid,self.missed = np.zeros(self.numberCameras,dtype=np.uint8),np.zeros(self.numberCameras,dtype=np.uint8)
        self.df = np.zeros((self.nImages,self.numberCameras*6+1))
        self.df[:,-1] = np.linspace(0,self.recTime,self.nImages)

    # CONNECT WITH CLIENTS
    def connect(self):
        print("[INFO] server running, waiting for clients")
        for i in range(self.numberCameras):
            # COLLECT ADDRESSES
            message,address = self.server_socket.recvfrom(self.bufferSize)
            self.imageSize.append(np.array(message.decode("utf-8").split(",")).astype(np.int))
            self.addDic[address[0]]=i
            self.myIPs.append(address)
            print('[INFO] client '+str(len(self.addDic))+' connected at '+str(address[0]))
            ret,newCamMatrix=self.myIntrinsics(self.cameraMat[i],self.imageSize[i][0],self.imageSize[i][1],self.imageSize[i][2])
            if ret: self.cameraMat[i]=np.copy(newCamMatrix)
            else: break
        # SEND TRIGGER 
        print('[INFO] all clients connected')
        self.triggerTime += time.time()
        for i in range(self.numberCameras): self.server_socket.sendto((str(self.triggerTime)+' '+str(self.recTime)).encode(),tuple(self.myIPs[i]))
        print('[INFO] trigger sent')

    # INTERPOLATE TO COMMON BASE
    def myInterpol(self,t,x,tNew):
        ff = CubicSpline(t, x,axis=0)
        return ff(tNew*self.step)

    # NEW INTRINSICS
    def myIntrinsics(self,intrisicsMatrix,w,h,mode):
        camIntris = np.copy(intrisicsMatrix) # copy to avoid registrer error
        # check if image is at the vailable proportion
        if w/h==4/3 or w/h==16/9:
            if mode==4: # only resize
                ratio = w/960
                camIntris[0][0],camIntris[0][2]=ratio*camIntris[0][0],ratio*camIntris[0][2]
                camIntris[1][1],camIntris[1][2]=ratio*camIntris[1][1],ratio*camIntris[1][2]
            elif mode==5: # crop in X and resize
                ratio = 1640/960
                camIntris[0][0],camIntris[0][2]=ratio*camIntris[0][0],ratio*camIntris[0][2]
                camIntris[1][1],camIntris[1][2]=ratio*camIntris[1][1],ratio*camIntris[1][2]-155
                ratio = w/1640
                camIntris[0][0],camIntris[0][2]=ratio*camIntris[0][0],ratio*camIntris[0][2]
                camIntris[1][1],camIntris[1][2]=ratio*camIntris[1][1],ratio*camIntris[1][2]
            elif mode==6: # crop in Y and X and resize
                ratio=1640/960
                camIntris[0][0],camIntris[0][2]=ratio*camIntris[0][0],ratio*camIntris[0][2]-180
                camIntris[1][1],camIntris[1][2]=ratio*camIntris[1][1],ratio*camIntris[1][2]-255
                ratio = w/1280
                camIntris[0][0],camIntris[0][2]=ratio*camIntris[0][0],ratio*camIntris[0][2]
                camIntris[1][1],camIntris[1][2]=ratio*camIntris[1][1],ratio*camIntris[1][2]
            elif mode==7: # crop in Y and X and resize
                ratio=1640/960
                camIntris[0][0],camIntris[0][2]=ratio*camIntris[0][0],ratio*camIntris[0][2]-500
                camIntris[1][1],camIntris[1][2]=ratio*camIntris[1][1],ratio*camIntris[1][2]-375
                ratio = w/640
                camIntris[0][0],camIntris[0][2]=ratio*camIntris[0][0],ratio*camIntris[0][2]
                camIntris[1][1],camIntris[1][2]=ratio*camIntris[1][1],ratio*camIntris[1][2]
            else:
                print('conversion for intrinsics matrix not known')
                return False,camIntris
            print('[INFO] resized intrisics')
            return True,camIntris
        else:
            print('out of proportion of the camera mode')
            return False,camIntris

    # RESHAPE THE COORDINATE VECTOR AND CHANGE ORDER IF CAMERAS ARE IDENTIFYING THE OPOSITE DIRECTION   
    def myReshaping(self,coord,tol=15):
        # get Y and X distances between markers
        centerX, centerY = np.vstack((coord[:,0], coord[:,2], coord[:,4])).T,np.vstack((coord[:,1], coord[:,3], coord[:,5])).T
        distX = np.array(centerX).max(axis=1) - np.array(centerX).min(axis=1)
        if np.all(distX<tol): maxVec,minVec = centerY[:,2],centerY[:,0]
        elif np.any(distX<tol):
            # looking to the arena, behind the cameras (where the AC is directed to the wall)
            # if lowest marker is from left camera, it is the foremost left marker of the right camera
            # if lowest marker is from right camera, it is the foremost right marker of the left camera
            idxY = np.argmax(distX<tol)
            lowestMarker = np.argmax(centerY[idxY])
            if not idxY: # ALWAYS USE CAMERA LEFT AS CAMERA 0
                leftMarker = np.argmin(centerX[1])
                if lowestMarker == leftMarker: return False
                return True
            else: 
                rightMarker = np.argmax(centerX[0])
                if lowestMarker == rightMarker: return False
                return True
        else: maxVec,minVec = centerX[:,2],centerX[:,0]
        # get signal between markers extremities
        firstCam,secCam = getSignal(maxVec[0],minVec[0]),getSignal(maxVec[1],minVec[1])
        if firstCam != secCam: return True
        return False

    # COLLECT POINTS FROM CLIENTS, ORDER AND TRIGGER INTERPOLATION
    def collect(self):
        # internal variables
        print('[INFO] waiting capture')
        counter,lastTime = np.zeros(self.numberCameras,dtype=np.uint16),np.zeros(self.numberCameras,dtype=np.uint32)
        swap,certainty = np.zeros(self.numberCameras,dtype=np.uint16),np.zeros(self.numberCameras,dtype=np.bool8)
        lastImgNumber,tol = np.zeros(self.numberCameras,dtype=np.int32),0.25
        intervals,timeIntervals,dataSave = [],[],[]
        for i in range(self.numberCameras): 
            intervals.append([])
            timeIntervals.append([])
            dataSave.append([])
        # capture loop
        try:
            while np.any(self.capture):
                #  RECEIVE MESSAGE
                bytesPair = self.server_socket.recvfrom(self.bufferSize)
                message = np.frombuffer(bytesPair[0],dtype=np.float64)
                address,sizeMsg = bytesPair[1],len(message)
                idx = self.addDic[address[0]]
                if not (sizeMsg-1): self.capture[idx] = 0

                if self.capture[idx]: # check if message is valid
                    if sizeMsg < 13: # if less than 3 blobs, discard
                        if self.verbose: print('[ERROR] '+str(int((sizeMsg-4)/3))+' markers were found')
                        self.invalid[idx]+=1
                    else: 
                        msg = message[0:sizeMsg-4].reshape(-1,3)
                        coord,size = msg[:,0:2],msg[:,2].reshape(-1)
                        # if more than 3 blobs are found, get the four biggest
                        if sizeMsg > 13: 
                            orderAscDiameters = np.argsort(size)
                            coord = np.array([coord[orderAscDiameters[-1]],coord[orderAscDiameters[-2]],coord[orderAscDiameters[-3]]]).reshape(-1,2)
                        # store message parameters
                        a,b,time,imgNumber = message[-4],message[-3],message[-2],int(message[-1]) 
                        # undistort points
                        if address[0] == '192.168.0.103': undCoord = processCentroids_calib(coord,a,b,self.cameraMat[0],distCoef_cam1)
                        else: undCoord = processCentroids_calib(coord,a,b,self.cameraMat[1],distCoef_cam2)
                        # check if there is an obstruction between the blobs
                        for [A,B,C] in undCoord.reshape([-1, 3, 2]):
                            if np.linalg.norm(A-B)<(size[0]+size[1])/2 or np.linalg.norm(A-C)<(size[0]+size[2])/2 or np.linalg.norm(B-C)<(size[1]+size[2])/2: 
                                if self.verbose: print('occlusion')
                                self.missed[idx]+=1
                                self.invalid[idx]+=1
                                continue
                        if self.save: dataSave[idx].append(np.concatenate((undCoord.reshape(6),[time,imgNumber])))
                        # if ts if not read corectly, discard
                        if counter[idx]:
                            if (time-lastTime[idx])>1e9: 
                                if self.verbose: print('time missmatch')
                                print(time,lastTime[idx])
                                self.missed[idx]+=1
                                self.invalid[idx]+=1
                                continue
                        # check if sequence is valid
                        if imgNumber>lastImgNumber[idx]+1: self.invalid[idx] = imgNumber-lastImgNumber[idx]
                        # order markers per proximity and check collinearity
                        if isCollinear(*undCoord) and not isEqual(undCoord,5) and not np.any(undCoord<0):     
                            if self.invalid[idx]>=10 or not counter[idx]: 
                                if certainty[idx]:
                                    beg,end = intervals[idx][-1],counter[idx]-1
                                    timeIntervals[idx].append([self.data[idx][beg][6],self.data[idx][end][6]])
                                    print('camera #'+str(idx)+' valid from '+str(round(self.data[idx][beg][6]/1e6,2))+'s to '+str(round(self.data[idx][end][6]/1e6,2))+'s')
                                prev,certainty[idx] = [],False
                                intervals[idx].append(counter[idx])
                            else: prev = np.array(self.data[idx][-1][0:6]).reshape(1,-2)
                            undCoord, _ = orderCenterCoord(undCoord,prev)
                            undCoord = np.array(undCoord)
                        else: 
                            if self.verbose: print('not collinear or equal centroids')
                            self.missed[idx]+=1
                            self.invalid[idx]+=1
                            continue
                        # update loop variables
                        lastTime[idx],lastImgNumber[idx],self.invalid[idx] = time,imgNumber,0
                        counter[idx]+=1
                        self.data[idx].append(np.hstack((undCoord.reshape(6),time)))
                        # check if ABC is in order smaller to largest
                        if not certainty[idx]:
                            for [A,B,C] in undCoord.reshape([-1, 3, 2]):
                                if np.linalg.norm(A-B)/np.linalg.norm(C-B)>(2-tol) and np.linalg.norm(A-B)>20:
                                    swap[idx] += 1
                                    if swap[idx]>2: 
                                        swap[idx],certainty[idx] = 0,True
                                        self.data[idx][intervals[idx][-1]:counter[idx]][0:2],self.data[idx][intervals[idx][-1]:counter[idx]][4:6]=self.data[idx][intervals[idx][-1]:counter[idx]][4:6],self.data[idx][intervals[idx][-1]:counter[idx]][0:2]
                                if np.linalg.norm(C-B)/np.linalg.norm(A-B)>(2-tol) and np.linalg.norm(C-B)>20: certainty[idx] = True
        finally:        
            # close everything
            self.server_socket.close()
            destroyAllWindows()
            # last interpolation loop
            for idx in range(self.numberCameras):
                if certainty[idx]:
                    beg,end = intervals[idx][-1],counter[idx]-1
                    timeIntervals[idx].append([self.data[idx][beg][6],self.data[idx][end][6]])    
                self.data[idx] = np.array(self.data[idx])
            # compute valid time intersection for interpolation
            intersections = [[max(first[0], second[0]), min(first[1], second[1])]  
                                for first in timeIntervals[0] for second in timeIntervals[1]  
                                if max(first[0], second[0]) <= min(first[1], second[1])]
            # interpolate at intersections
            for [beg,end] in intersections:
                for idx in range(self.numberCameras):
                    validIdx = [i for i in range(0,len(self.data[idx])) if beg<=self.data[idx][i][-1]<=end]
                    coord,time = self.data[idx][validIdx,0:6],self.data[idx][validIdx,6]/1e6
                    lowBound,highBound = math.ceil(time[0]/self.step),math.floor(time[-1]/self.step)
                    print('interpolated #'+str(idx+1)+' from '+str(round(lowBound*self.step,2))+'s to '+str(round(highBound*self.step,2))+'s')
                    tNew = np.linspace(lowBound,highBound,int((highBound-lowBound))+1,dtype=np.uint16)
                    if time.shape[0]<=2: continue
                    ff = CubicSpline(time,coord,axis=0)
                    self.df[tNew,int(idx*6):int(idx*6+6)] = ff(tNew*self.step)
            # save centroids
            self.df = np.delete(self.df,np.unique([i for i in range(0,self.df.shape[0]) for idx in range(self.numberCameras) if not np.any(self.df[i][idx*6:idx*6+6])]),axis=0)
            centroids1,centroids2 = self.df[:,0:6].reshape(-1,2),self.df[:,6:12].reshape(-1,2)
            # get fundamental and essential matrices
            R,t,_ = self.computeFandE(centroids1,centroids2)
            # print results
            print('[RESULTS] server results are')
            for i in range(self.numberCameras): print('  >> camera '+str(i)+': '+str(len(self.data[i]))+' captured valid images images, address '+str(self.myIPs[i][0])+', missed '+str(int(self.missed[i]))+' images')
            #plot the beautiful stuff
            if self.save: 
                np.savetxt('cam1.csv', np.array(dataSave[0]), delimiter=',')
                np.savetxt('cam2.csv', np.array(dataSave[1]), delimiter=',')
            self.plotDataset()
            _,_,falseIdx = self.plotStd(centroids1,centroids2,R,t)
            # remove unwanted centroids
            centroids1,centroids2=np.delete(centroids1,falseIdx,axis=0),np.delete(centroids2,falseIdx,axis=0)
            R,t,F = self.computeFandE(centroids1,centroids2)
            lamb,points3d,_ = self.plotStd(centroids1,centroids2,R,t)
            self.plot3D(R,t,lamb,points3d)
            self.plotEpilines(centroids1,centroids2,F)

    # get fundamental and essential matrices
    def computeFandE(self,centroids1,centroids2):
        F,_ = estimateFundMatrix_8norm(np.array(centroids1),np.array(centroids2))
        E = np.matmul(cameraMatrix_cam2.T, np.matmul(F, cameraMatrix_cam1))
        print("\nEssenc. Mat.\n", E.round(4))
        # decompose to rotation and translation between cameras
        R, t = decomposeEssentialMat(E, cameraMatrix_cam1, cameraMatrix_cam2, np.array(centroids1), np.array(centroids2))
        if np.any(np.isnan(R)): print('no valid rotation matrix')
        else:
            print("\nRot. Mat.\n", R.round(4))
            print("\nTrans. Mat.\n", t.round(4))
        return R,t,F

    # plot functions
    def plotDataset(self):
        # dataset recap
        for i in range(self.numberCameras):       
            _,axs = plt.subplots(3, 2,figsize=(10,20),dpi=100)
            axs[0,0].plot(np.array(self.data[i])[:,6]/1e6, np.array(self.data[i])[:,0], 'o',label='40 FPS') 
            axs[0,0].plot(self.df[:,-1], self.df[:,i*6], 'D',label='100 FPS interpolation')
            axs[0,0].set_ylabel('X - marker #0 (px)')
            axs[0,0].grid()
            axs[0,1].plot(np.array(self.data[i])[:,6]/1e6, np.array(self.data[i])[:,1], 'o',label='40 FPS') 
            axs[0,1].plot( self.df[:,-1], self.df[:,i*6+1], 'D',label='100 FPS interpolation')
            axs[0,1].set_ylabel('Y - marker #0 (px)')
            axs[0,1].grid()
            axs[1,0].plot(np.array(self.data[i])[:,6]/1e6, np.array(self.data[i])[:,2], 'o',label='40 FPS') 
            axs[1,0].plot( self.df[:,-1], self.df[:,i*6+2], 'D',label='100 FPS interpolation')
            axs[1,0].set_ylabel('X - marker #1 (px)')
            axs[1,0].grid()
            axs[1,1].plot(np.array(self.data[i])[:,6]/1e6, np.array(self.data[i])[:,3], 'o',label='40 FPS') 
            axs[1,1].plot(self.df[:,-1], self.df[:,i*6+3], 'D',label='100 FPS interpolation')
            axs[1,1].set_ylabel('X - marker #1 (px)')
            axs[1,1].grid()
            axs[2,0].plot(np.array(self.data[i])[:,6]/1e6, np.array(self.data[i])[:,4], 'o',label='40 FPS') 
            axs[2,0].plot(  self.df[:,-1], self.df[:,i*6+4], 'D',label='100 FPS interpolation')
            axs[2,0].set_xlabel('t (seconds)')
            axs[2,0].set_ylabel('X - marker #2 (px)')
            axs[2,0].grid()
            axs[2,1].plot(np.array(self.data[i])[:,6]/1e6, np.array(self.data[i])[:,5], 'o',label='40 FPS') 
            axs[2,1].plot(  self.df[:,-1], self.df[:,i*6+5], 'D',label='100 FPS interpolation')
            axs[2,1].set_xlabel('t (seconds)')
            axs[2,1].set_ylabel('Y - marker #2 (px)')
            axs[2,1].grid()
            plt.draw()
            plt.show()
    
    def plotStd(self,centroids1,centroids2,R,t):
        # first projection computation
        P1,P2 = np.hstack((cameraMatrix_cam1, [[0.], [0.], [0.]])),np.matmul(cameraMatrix_cam2, np.hstack((R, t.T)))
        projPt1,projPt2 = myProjectionPoints(np.array(centroids1)),myProjectionPoints(np.array(centroids2))
        points4d = triangulatePoints(P1.astype(float),P2.astype(float),projPt1.astype(float),projPt2.astype(float))
        points3d = (points4d[:3, :]/points4d[3, :]).T
        if points3d[0, 2] < 0: points3d = -points3d
        tot,L_real_AC,L_real_AB,L_real_BC,L_AC_vec,L_BC_vec,L_AB_vec,k,false_idx = 0,15.7,5.5,10.2,[],[],[],0,[]
        # compute sdt deviation and plot beautiful stuff
        for [A, B, C] in points3d.reshape([-1, 3, 3]):
            L_rec_AC,L_rec_BC,L_rec_AB = np.linalg.norm(A-C),np.linalg.norm(B-C),np.linalg.norm(A-B)
            if L_rec_AB<L_rec_BC: L_rec_AB,L_rec_BC=L_rec_BC,L_rec_AB
            tot = tot + L_real_AC/L_rec_AC + L_real_BC/L_rec_BC + L_real_AB/L_rec_AB
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
        L_AC_vec_plot,L_BC_vec_plot,L_AB_vec_plot = np.array(L_AC_vec)*lamb - L_real_AC,np.array(L_BC_vec)*lamb - L_real_BC,np.array(L_AB_vec)*lamb - L_real_AB
        plt.plot(L_AC_vec_plot, '-o', label="std_AC")
        plt.plot(L_BC_vec_plot, '-o', label="std_BC")
        plt.plot(L_AB_vec_plot, '-o', label="std_AB")
        plt.axhline(y=0.0, color='r', linestyle='-')
        plt.grid()
        plt.xlabel("Image number")
        plt.ylabel("Deviation to mean value (cm)")
        plt.legend()
        ax = fig.axes
        ax[0].minorticks_on()
        plt.grid(which='both')
        plt.xlim(0,len(L_AC_vec)-1)
        # get the bad triangulated points
        points3d_new,i,k= points3d*lamb,0,0
        for [A, B, C] in points3d_new.reshape([-1, 3, 3]):
            L_reconst = np.sqrt(np.sum((A-C)**2, axis=0))
            valid = abs(L_real_AC-L_reconst)/L_real_AC < 0.01
            if not valid: 
                i = i + 1
                false_idx.extend((k,k+1,k+2))
            k+=3
        print("Images distant more than 1% from the real value = " + str(i)+'/'+str(int(points3d.shape[0]/3)))
        plt.draw()
        plt.show() 
        return lamb,points3d,false_idx

    def plot3D(self,R,t,lamb,points3d):
        # plot 3d map
        fig = plt.figure(figsize=(8, 8))
        ax = plt.axes(projection='3d')
        ax.set_xlim(-0.1, 3.1)
        ax.set_zlim(-0.4, 0.4)
        ax.set_ylim(-0.1, 4)
        ax.set_xlabel('X')
        ax.set_ylabel('Z')
        ax.set_zlabel('Y')
        scale = 0.3
        x,y,z = np.array([scale, 0, 0]), np.array([0, scale, 0]),np.array([0, 0, scale])
        # rotate second camera to the same plane
        t_aux = np.matmul(-t, R)*lamb/100
        theta = np.arctan2(t_aux[0][1],t_aux[0][0])
        Rz = np.array([[np.cos(-theta),np.sin(-theta),0], [np.sin(-theta), np.cos(-theta),0], [0,0,1]])
        x,y,z = np.matmul(Rz, x),np.matmul(Rz, y),np.matmul(Rz, z)
        ax.quiver(0, 0, 0, x[0], x[2], x[1], arrow_length_ratio=0.1, edgecolors="r")
        ax.quiver(0, 0, 0, y[0], y[2], y[1], arrow_length_ratio=0.1, edgecolors="b")
        ax.quiver(0, 0, 0, z[0], z[2], z[1], arrow_length_ratio=0.1, edgecolors="g")
        ax.scatter(0, 0, 0, edgecolor="blue", facecolor="black")
        x,y,z = np.array([scale, 0, 0]), np.array([0, scale, 0]),np.array([0, 0, scale])
        x,y,z = np.matmul(R.T, x),np.matmul(R.T, y),np.matmul(R.T, z)
        x,y,z = np.matmul(Rz, x),np.matmul(Rz, y),np.matmul(Rz, z)
        t_new,points3d_new = np.matmul(Rz,t_aux.T).T,np.matmul(Rz,points3d.T).T
        ax.quiver(t_new[0][0], t_new[0][2], t_new[0][1], x[0], x[2], x[1], arrow_length_ratio=0.1, edgecolors="r")
        ax.quiver(t_new[0][0], t_new[0][2], t_new[0][1], y[0], y[2], y[1], arrow_length_ratio=0.1, edgecolors="b")
        ax.quiver(t_new[0][0], t_new[0][2], t_new[0][1], z[0], z[2], z[1], arrow_length_ratio=0.1, edgecolors="g")
        ax.scatter(t_new[0][0], t_new[0][2], t_new[0][1], edgecolor="r", facecolor="gold")
        cmhot = plt.get_cmap("jet")
        ax.view_init(elev=15, azim=-30)  
        ax.scatter(points3d_new[:, 0]*lamb/100, points3d_new[:, 2]*lamb/100, points3d_new[:, 1]*lamb/100, c=points3d_new[:, 2], cmap=cmhot)
        plt.gca().invert_zaxis()
        ax.get_proj = lambda: np.dot(Axes3D.get_proj(ax), np.diag([1., 1., .3, 1.]))
        plt.draw()
        plt.show()

    def plotEpilines(self,centroids1,centroids2,F):
        img1,img2 = np.ones((540,960))*255,np.ones((540,960))*255
        pts1,pts2 = np.int32(centroids1[0:3].reshape(-1,2)),np.int32(centroids2[0:3].reshape(-1,2))

        def drawlines(img1,img2,lines,pts1,pts2):
            ''' img1 - image on which we draw the epilines for the points in img2
                lines - corresponding epilines '''
            r,c = img1.shape
            img1 = cvtColor(img1.astype('float32'),COLOR_GRAY2BGR)
            img2 = cvtColor(img2.astype('float32'),COLOR_GRAY2BGR)
            for r,pt1,pt2 in zip(lines,pts1,pts2):
                color = (0,0,0)
                x0,y0 = map(int, [0, -r[2]/r[1] ])
                x1,y1 = map(int, [c, -(r[2]+r[0]*c)/r[1] ])
                img1 = line(img1, (x0,y0), (x1,y1), color,1)
                img1 = circle(img1,tuple(pt1),5,color,-1)
                img2 = circle(img2,tuple(pt2),5,color,-1)
            return img1,img2

        lines1 = computeCorrespondEpilines(pts2.reshape(-1,1,2), 2,F)
        lines1 = lines1.reshape(-1,3)
        img5,_ = drawlines(img1,img2,lines1,pts1,pts2)
        lines2 = computeCorrespondEpilines(pts1.reshape(-1,1,2), 1,F)
        lines2 = lines2.reshape(-1,3)
        img3,_ = drawlines(img2,img1,lines2,pts2,pts1)
        plt.figure(figsize=(20, 16))
        plt.subplot(121),plt.imshow(img5)
        plt.subplot(122),plt.imshow(img3)
        plt.show()
 
            
# parser for command line
parser = argparse.ArgumentParser(description='''Server for the MoCap system at the Erobotica lab of UFCG.
                                                \nPlease use it together with the corresponding client script.''',add_help=False)
parser.add_argument('-n',type=int,help='number of cameras (default: 2)',default=2)
parser.add_argument('-trig',type=int,help='trigger time in seconds (default: 10)',default=10)
parser.add_argument('-rec',type=int,help='recording time in seconds (default: 30)',default=30)
parser.add_argument('-fps',type=int,help='interpolation fps (default: 100FPS)',default=100)
parser.add_argument('--verbose',help='show points capture after end of recording time (default: off)',default=False, action='store_true')
parser.add_argument('--help', action='help', default=argparse.SUPPRESS, help='show this help message and exit.')
parser.add_argument('-out', help='write the rotation matrix and translation vector to filename (default: off)',default=False, action='store_true')
parser.add_argument('-save',help='save received packages to CSV (default: off)',default=False, action='store_true')
args = parser.parse_args()

myServer_ = myServer(args.n,args.trig,args.rec,args.fps,args.verbose,args.out,args.save)
myServer_.connect()
myServer_.collect()