# IMPORTS >>> DO NOT CHANGE <<<
import warnings
warnings.filterwarnings("ignore")
import socket,time,math,argparse
import numpy as np
from cv2 import circle,destroyAllWindows,triangulatePoints,cvtColor,line,COLOR_GRAY2BGR,computeCorrespondEpilines
from myLib import orderCenterCoord,estimateFundMatrix_8norm,decomposeEssentialMat,myProjectionPoints,isCollinear,isEqual,processCentroids
from constants import cameraMat,distCoef
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
from mpl_toolkits.mplot3d import Axes3D

class myServer(object):
    def __init__(self,numberCameras,triggerTime,recTime,FPS,verbose,output,save,ipList):
        ##########################################
        # PLEASE DONT CHANGE THE VARIABLES BELOW #
        ##########################################
        self.numberCameras,self.triggerTime,self.recTime,self.step,self.out,self.save,self.verbose,self.ipList = numberCameras,triggerTime,recTime,1/FPS,output,save,verbose,np.array(ipList.split(','))
        # check number of ips
        if self.ipList.shape[0]!=self.numberCameras:
            print('[ERROR] Number of cameras do not match the number of IPs given')
            exit()
        self.cameraMat,self.distCoef = np.copy(cameraMat),np.copy(distCoef)
        # do not change below this line, socket variables
        self.nImages,self.imageSize = int(self.recTime/self.step),[]
        for i in range(self.numberCameras): self.imageSize.append([])
        print('[INFO] creating server')
        self.bufferSize,self.server_socket = 1024,socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        self.server_socket.bind(('0.0.0.0',8888))

    # CONNECT WITH CLIENTS
    def connect(self):
        print("[INFO] server running, waiting for clients")
        addedCams,ports,k=[],[],0
        while k!=self.numberCameras:
            # COLLECT ADDRESSES
            message,address = self.server_socket.recvfrom(self.bufferSize)
            # CHECK IF IS IN IP LIST
            if not len(np.where(self.ipList==address[0])[0]):
                print('[ERROR] IP '+address[0]+' not in the list')
                exit()
            # GET IMAGE SIZE
            idx = np.where(self.ipList==address[0])[0][0]
            if idx in addedCams: continue
            self.imageSize[idx] = np.array(message.decode("utf-8").split(",")).astype(np.int)
            print('[INFO] camera '+str(idx)+' connected at '+str(address[0]))
            # REDO INTRINSICS
            ret,newCamMatrix=self.myIntrinsics(self.cameraMat[idx],self.imageSize[idx][0],self.imageSize[idx][1],self.imageSize[idx][2])
            if ret: self.cameraMat[idx]=np.copy(newCamMatrix)
            else: exit()
            k+=1
            addedCams.append(idx)
            ports.append(address)
        # SEND TRIGGER 
        print('[INFO] all clients connected')
        self.triggerTime += time.time()
        for i in range(self.numberCameras): self.server_socket.sendto((str(self.triggerTime)+' '+str(self.recTime)).encode(),tuple(ports[i]))
        print('[INFO] trigger sent')

    # NEW INTRINSICS
    def myIntrinsics(self,origMatrix,w,h,mode):
        camIntris = np.copy(origMatrix) # copy to avoid registrer error
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
                print('[ERROR] conversion for intrinsics matrix not known')
                return False,camIntris
            return True,camIntris
        else:
            print('[ERROR] out of proportion of the camera mode')
            return False,camIntris

    # COLLECT POINTS FROM CLIENTS, ORDER AND TRIGGER INTERPOLATION
    def collect(self):
        # internal variables
        print('[INFO] waiting capture')
        capture = np.ones(self.numberCameras,dtype=np.bool)
        counter,lastTime = np.zeros(self.numberCameras,dtype=np.uint16),np.zeros(self.numberCameras,dtype=np.uint32)
        missed,invalid = np.zeros(self.numberCameras,dtype=np.uint32),np.zeros(self.numberCameras,dtype=np.uint32)
        swap,certainty = np.zeros(self.numberCameras,dtype=np.uint16),np.zeros(self.numberCameras,dtype=np.bool8)
        lastImgNumber,tol = np.zeros(self.numberCameras,dtype=np.int32),0.25
        intervals,timeIntervals,dfSave,dfOrig = [],[],[],[]
        rotation,translation,scale,FMatrix,points3D_perPair, = [np.identity(3)],[[[0., 0., 0.]]],[[1]],[],[]
        for i in range(self.numberCameras): 
            intervals.append([])
            timeIntervals.append([])
            dfOrig.append([])
        # capture loop
        try:
            while np.any(capture):
                #  receive message
                bytesPair = self.server_socket.recvfrom(self.bufferSize)
                message = np.frombuffer(bytesPair[0],dtype=np.float64)
                address,sizeMsg = bytesPair[1],len(message)
                idx = np.where(self.ipList==address[0])[0][0]
                if not (sizeMsg-1): capture[idx] = 0
                # if valid message
                if capture[idx]: # check if message is valid
                    if sizeMsg < 13: # if less than 3 blobs, discard
                        if self.verbose: print('[ERROR] '+str(int((sizeMsg-4)/3))+' markers were found')
                        missed[idx]+=1
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
                        undCoord = processCentroids(coord,a,b,self.cameraMat[idx],distCoef[idx])
                        # check if there is an obstruction between the blobs
                        '''for [A,B,C] in undCoord.reshape([-1, 3, 2]):
                            if np.linalg.norm(A-B)<(size[0]+size[1])/2 or np.linalg.norm(A-C)<(size[0]+size[2])/2 or np.linalg.norm(B-C)<(size[1]+size[2])/2: 
                                if self.verbose: print('occlusion')
                                self.missed[idx]+=1
                                self.invalid[idx]+=1
                                continue'''
                        if self.save: dfSave.append(np.concatenate((undCoord.reshape(6),[time,imgNumber,idx])))
                        # if ts if not read corectly, discard
                        if counter[idx]:
                            if abs(time-lastTime[idx])>1e9: 
                                if self.verbose: print('[WARNING-CAM+'+str(idx)+'] time missmatch')
                                missed[idx]+=1
                                invalid[idx]+=1
                                continue
                        # check if sequence is valid
                        if imgNumber>lastImgNumber[idx]+1: invalid[idx] = imgNumber-lastImgNumber[idx]
                        # order markers per proximity and check collinearity
                        if isCollinear(*undCoord) and not isEqual(undCoord,5) and not np.any(undCoord<0):     
                            if invalid[idx]>=10 or not counter[idx]: 
                                if certainty[idx]:
                                    beg,end = intervals[idx][-1],counter[idx]-1
                                    timeIntervals[idx].append([dfOrig[idx][beg,6],dfOrig[idx][end,6]])
                                    if self.verbose: print('[WARNING-CAM+'+str(idx)+'] valid from '+str(round(dfOrig[idx][beg,6]/1e6,2))+'s to '+str(round(dfOrig[idx][end][6]/1e6,2))+'s')
                                prev,certainty[idx] = [],False
                                intervals[idx].append(counter[idx])
                            else: 
                                if not (counter[idx]-1): prev = np.array(dfOrig[idx][0:6]).reshape(-1,2)
                                else: prev = np.array(dfOrig[idx][-1,0:6]).reshape(-1,2)
                            undCoord, _ = orderCenterCoord(undCoord,prev)
                            undCoord = np.array(undCoord)
                        else: 
                            if self.verbose: print('[WARNING-CAM+'+str(idx)+'] not collinear or equal centroids')
                            missed[idx]+=1
                            invalid[idx]+=1
                            continue
                        # update loop variables
                        lastTime[idx],lastImgNumber[idx],invalid[idx] = time,imgNumber,0
                        if not counter[idx]: dfOrig[idx] = np.hstack((undCoord.reshape(6),time))
                        else: dfOrig[idx] = np.vstack((dfOrig[idx],np.hstack((undCoord.reshape(6),time))))
                        counter[idx]+=1
                        # check if ABC is in order smaller to largest
                        if not certainty[idx]:
                            for [A,B,C] in undCoord.reshape([-1, 3, 2]):
                                if np.linalg.norm(A-B)/np.linalg.norm(C-B)>(2-tol) and np.linalg.norm(A-B)>20:
                                    swap[idx] += 1
                                    if swap[idx]>2:    
                                        swap[idx],certainty[idx] = 0,True
                                        dfOrig[idx][intervals[idx][-1]:counter[idx],0:2],dfOrig[idx][intervals[idx][-1]:counter[idx],4:6] = np.copy(dfOrig[idx][intervals[idx][-1]:counter[idx],4:6]),np.copy(dfOrig[idx][intervals[idx][-1]:counter[idx],0:2])
                                if np.linalg.norm(C-B)/np.linalg.norm(A-B)>(2-tol) and np.linalg.norm(C-B)>20:  certainty[idx] = True
                                            
        finally:        
            # close everything
            self.server_socket.close()
            destroyAllWindows()
            # get last interval
            for idx in range(self.numberCameras):
                if not len(dfOrig[idx]): continue
                if certainty[idx]:
                    beg,end = intervals[idx][-1],counter[idx]-1
                    timeIntervals[idx].append([dfOrig[idx][beg,6],dfOrig[idx][end,6]])    
                    if self.verbose: print('[WARNING-CAM+'+str(idx)+'] valid from '+str(round(dfOrig[idx][beg,6]/1e6,2))+'s to '+str(round(dfOrig[idx][end][6]/1e6,2))+'s')
            # print results
            print('[RESULTS] server results are')
            for i in range(self.numberCameras): print('  >> camera '+str(i)+': '+str(len(dfOrig[i]))+' valid images, address '+str(self.ipList[i])+', missed '+str(int(missed[i]))+' images')
            if self.save: np.savetxt('camCalib.csv', np.array(dfSave), delimiter=',')
            # get pose between each pair
            for j in range(self.numberCameras-1):
                # compute valid time intersection for interpolation
                intersections = [[max(first[0], second[0]), min(first[1], second[1])]  
                                    for first in timeIntervals[j] for second in timeIntervals[j+1]  
                                    if max(first[0], second[0]) <= min(first[1], second[1])]
                # create and fill inteprolation dataset based on the intersection of valid time intervals 
                dfInterp = np.zeros((self.nImages,2*6+1))
                dfInterp[:,-1] = np.arange(0,self.recTime,self.step)
                for [beg,end] in intersections:
                    for idx in range(j,j+2):
                        validIdx = [i for i in range(0,len(dfOrig[idx])) if beg<=dfOrig[idx][i,-1]<=end]
                        coord,time = dfOrig[idx][validIdx,0:6],dfOrig[idx][validIdx,6]/1e6
                        if time.shape[0]<=2: continue
                        lowBound,highBound = math.ceil(time[0]/self.step),math.floor(time[-1]/self.step)
                        if self.verbose: print('[INFO] interpolated #'+str(idx)+' from '+str(round(lowBound*self.step,2))+'s to '+str(round(highBound*self.step,2))+'s')
                        tNew = np.linspace(lowBound,highBound,int((highBound-lowBound))+1,dtype=np.uint16)
                        ff = CubicSpline(time,coord,axis=0)
                        dfInterp[tNew,int((idx-j)*6):int((idx-j)*6+6)] = ff(tNew*self.step)
                # get data from the interpolation dataset
                dfInterp = np.delete(dfInterp,np.unique([i for i in range(0,dfInterp.shape[0]) for idx in range(2) if not np.any(dfInterp[i][idx*6:idx*6+6])]),axis=0)
                if dfInterp.shape[0] < 10: 
                    print('[ERROR] no valid image intersection for cameras '+str(j)+' and '+str(j+1))
                    return
                centroids1,centroids2 = dfInterp[:,0:6].reshape(-1,2),dfInterp[:,6:12].reshape(-1,2)
                print('[WARNING] interpolated '+str(dfInterp.shape[0])+' images between cameras '+str(j)+' and '+str(j+1))
                # get fundamental and essential matrices 
                print('[INFO] Computing fundamental and essential matrix between cameras '+str(j)+'-'+str(j+1))
                try: 
                    F,_ = estimateFundMatrix_8norm(np.array(centroids1),np.array(centroids2),verbose=False)
                    E = np.matmul(self.cameraMat[j+1].T, np.matmul(F, self.cameraMat[j]))
                    # decompose to rotation and translation between cameras
                    R, t = decomposeEssentialMat(E, self.cameraMat[j], self.cameraMat[j+1], np.array(centroids1), np.array(centroids2))
                    if np.any(np.isnan(R)): 
                        print('[ERROR] no valid rotation matrix')
                        return
                    else:
                        if self.verbose:
                            print("\nRot. Mat.\n", R.round(4))
                            print("\nTrans. Mat.\n", t.round(4))
                except: 
                    print('[ERROR] no valid rotation matrix')
                    return
                # create projection matrices and triangulate to compute scale
                P1,P2 = np.hstack((self.cameraMat[j], [[0.], [0.], [0.]])),np.matmul(self.cameraMat[j+1], np.hstack((R, t.T)))
                projPt1,projPt2 = myProjectionPoints(np.array(centroids1)),myProjectionPoints(np.array(centroids2))
                points4d = triangulatePoints(P1.astype(float),P2.astype(float),projPt1.astype(float),projPt2.astype(float))
                points3d = (points4d[:3, :]/points4d[3, :]).T
                if points3d[0, 2] < 0: points3d = -points3d
                tot,L_real_AC,L_real_AB,L_real_BC,k,false_idx = 0,15.7,5.5,10.2,0,[]
                # compute sdt deviation and plot beautiful stuff
                for [A, B, C] in points3d.reshape([-1, 3, 3]):
                    L_rec_AC,L_rec_BC,L_rec_AB = np.linalg.norm(A-C),np.linalg.norm(B-C),np.linalg.norm(A-B)
                    tot = tot + L_real_AC/L_rec_AC + L_real_BC/L_rec_BC + L_real_AB/L_rec_AB
                    k+=3
                lamb = tot/k
                points3d_new,i,k= points3d*lamb,0,0
                for [A, B, C] in points3d_new.reshape([-1, 3, 3]):
                    L_reconst = np.sqrt(np.sum((A-C)**2, axis=0))
                    valid = abs(L_real_AC-L_reconst)/L_real_AC < 0.01
                    if not valid: 
                        i+=1
                        false_idx.extend((k,k+1,k+2))
                    k+=3
                print("\tImages distant more than 1% from the real value = " + str(i)+'/'+str(int(points3d.shape[0]/3)))
                # deleting points and refining estimation of the fundamental and essential matrices
                print("[INFO] Refining fundamental matrix estimation")
                centroids1,centroids2=np.delete(np.copy(centroids1),false_idx,axis=0),np.delete(np.copy(centroids2),false_idx,axis=0)
                # get fundamental and essential matrices
                F,_ = estimateFundMatrix_8norm(np.copy(centroids1),np.copy(centroids2))
                E = np.matmul(self.cameraMat[j+1].T, np.matmul(F, self.cameraMat[j]))
                # decompose to rotation and translation between cameras
                R, t = decomposeEssentialMat(E, self.cameraMat[j], self.cameraMat[j+1], np.copy(centroids1), np.copy(centroids2))
                if np.any(np.isnan(R)): 
                    print('[ERROR] no valid rotation matrix')
                    return
                else:
                    if self.verbose:
                        print("\nRot. Mat.\n", R.round(4))
                        print("\nTrans. Mat.\n", t.round(4))
                P1,P2 = np.hstack((self.cameraMat[j], [[0.], [0.], [0.]])),np.matmul(self.cameraMat[j+1], np.hstack((R, t.T)))
                projPt1,projPt2 = myProjectionPoints(np.copy(centroids1)),myProjectionPoints(np.copy(centroids2))
                points4d = triangulatePoints(P1.astype(float),P2.astype(float),projPt1.astype(float),projPt2.astype(float))
                points3d = (points4d[:3, :]/points4d[3, :]).T
                if points3d[0, 2] < 0: points3d = -points3d
                tot,L_AC_vec,L_BC_vec,L_AB_vec,k = 0,[],[],[],0
                # compute sdt deviation and plot beautiful stuff
                for [A, B, C] in points3d.reshape([-1, 3, 3]):
                    L_rec_AC,L_rec_BC,L_rec_AB = np.linalg.norm(A-C),np.linalg.norm(B-C),np.linalg.norm(A-B)
                    tot = tot + L_real_AC/L_rec_AC + L_real_BC/L_rec_BC + L_real_AB/L_rec_AB
                    k+=3
                    L_AC_vec.append(L_rec_AC)
                    L_BC_vec.append(L_rec_BC)
                    L_AB_vec.append(L_rec_AB)
                lamb = tot/k
                print('\tScale between real world and triang. point cloud is: ', lamb.round(2))
                print('\tL_AC >> mean = ' + str((np.mean(L_AC_vec)*lamb).round(4)) +
                    "cm, std. dev = " + str((np.std(L_AC_vec)*lamb).round(4)) +
                    "cm, rms = " + str((np.sqrt(np.mean(np.square(np.array(L_AC_vec)*lamb-L_real_AC)))).round(4)) + "cm")
                print('\tL_AB >> mean = ' + str((np.mean(L_AB_vec)*lamb).round(4)) +
                    "cm, std. dev = " + str((np.std(L_AB_vec)*lamb).round(4)) +
                    "cm, rms = " + str((np.sqrt(np.mean(np.square(np.array(L_AB_vec)*lamb-L_real_AB)))).round(4)) + "cm")
                print('\tL_BC >> mean = ' + str((np.mean(L_BC_vec)*lamb).round(4)) +
                    "cm, std. dev = " + str((np.std(L_BC_vec)*lamb).round(4)) +
                    "cm, rms = " + str((np.sqrt(np.mean(np.square(np.array(L_BC_vec)*lamb-L_real_BC)))).round(4)) + "cm")
                translation.append(t)
                rotation.append(R)
                scale.append([lamb])
                FMatrix.append(F)
                points3D_perPair.append(points3d)
            # saving csv
            np.savetxt('R.csv', np.ravel(rotation), delimiter=',')
            np.savetxt('t.csv', np.ravel(translation), delimiter=',')
            np.savetxt('lamb.csv', np.ravel(scale), delimiter=',')
            np.savetxt('F.csv', np.ravel(FMatrix), delimiter=',')
            # setting 3D plot variables
            fig = plt.figure(figsize=(8, 8),dpi=100)
            ax = plt.axes(projection='3d')
            ax.set_xlim(-1, 4)
            ax.set_zlim(-6, 0)
            ax.set_ylim(-1, 4)
            ax.set_xlabel('X')
            ax.set_ylabel('Z')
            ax.set_zlabel('Y')
            ax.set_xlabel('X', fontweight='bold',labelpad=15)
            ax.set_ylabel('Z', fontweight='bold',labelpad=15)
            ax.set_zlabel('Y', fontweight='bold',labelpad=5)
            cmhot = plt.get_cmap("jet")
            ax.view_init(elev=30, azim=-50) 
            plt.gca().invert_zaxis()
            ax.get_proj = lambda: np.dot(Axes3D.get_proj(ax), np.diag([1., 1., .5, 1.]))
            colours = [['fuchsia','plum'],['darkorange','gold'],['limegreen','greenyellow'],['blue','lightsteelblue']]
            # initializing arrays
            allPts3d,projMat = [],[]            
            # getting coordinates from each camera and 3D points in relation to the 0th camera
            for j in range(self.numberCameras):
                # initialize initial values of the projection matrices
                x,y,z= np.array([1, 0, 0, 0]), np.array([0, 1, 0, 0]),np.array([0, 0, 1, 0])
                P_new = np.vstack((np.hstack((np.identity(3),np.zeros((3,1)))),np.hstack((np.zeros((3)),1))))
                # iterate over the previous cameras to find the relation to the 0th camera
                # e.g. 3 -> 2 -> 1 -> 0
                for i in np.flip(range(j+1)):
                    t,R,lamb = np.array(translation[i][0]).reshape(-1,3),np.array(rotation[i]),scale[i]
                    t_new = np.matmul(-t, R).reshape(-1,3)*lamb/100
                    P = np.vstack((np.hstack((R.T,t_new.T)),np.hstack((np.zeros((3)),1))))
                    P_new = np.matmul(P,P_new)
                # save new projection matrix
                projMat.append(P_new)
                # plot camera pose
                x,y,z = np.matmul(P_new,x),np.matmul(P_new,y),np.matmul(P_new,z)
                o = np.matmul(P_new,[[0.],[0.],[0.],[1]]).ravel()
                ax.quiver(o[0], o[2], o[1], x[0], x[2], x[1], arrow_length_ratio=0.05, edgecolors="r", label='X axis')
                ax.quiver(o[0], o[2], o[1], y[0], y[2], y[1], arrow_length_ratio=0.05, edgecolors="b", label='Y axis')
                ax.quiver(o[0], o[2], o[1], z[0], z[2], z[1], arrow_length_ratio=0.05, edgecolors="g", label='Z axis')
                ax.scatter(o[0], o[2], o[1], s=50, edgecolor=colours[j][0], facecolor=colours[j][1], linewidth=2,  label = 'Camera '+str(j))
                # save triangulated 3D points in relation to the 0th camera
                if j<(self.numberCameras-1):
                    points3d = np.hstack((points3D_perPair[j]*scale[j+1][0]/100,np.ones((points3D_perPair[j].shape[0],1)))).T
                    points3d = np.matmul(P_new,points3d)
                    ax.scatter(points3d[0], points3d[2], points3d[1], s=50, c=points3d[2], cmap=cmhot)
                    if not len(allPts3d): allPts3d = points3d.copy()
                    else: allPts3d = np.hstack((allPts3d,points3d.copy()))
            # saving projection matrices
            np.savetxt('projMat.csv', np.array(projMat).ravel(), delimiter=',')
            # final plot commands
            handles, labels = plt.gca().get_legend_handles_labels()
            by_label = dict(zip(labels, handles))
            plt.legend(by_label.values(), by_label.keys(),ncol=3,loc ='center',edgecolor='silver', bbox_to_anchor=(0.5, 0.8))
            plt.title('3D map of the calibrated arena')
            plt.draw()
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
parser.add_argument('-out', help='write the rotation matrix and translation vector(default: off)',default=False, action='store_true')
parser.add_argument('-save',help='save received packages to CSV (default: off)',default=False, action='store_true')
parser.add_argument('-ip',type=str,help='List of IPs of each camera, in order',default='')
args = parser.parse_args()

myServer_ = myServer(args.n,args.trig,args.rec,args.fps,args.verbose,args.out,args.save,args.ip)
myServer_.connect()
myServer_.collect()