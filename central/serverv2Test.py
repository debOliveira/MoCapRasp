# IMPORTS >>> DO NOT CHANGE <<<
import warnings
warnings.filterwarnings("ignore")
import socket,time,math,argparse
import numpy as np
from functions import processCentroids_calib,cameraMatrix_cam1,cameraMatrix_cam2,distCoef_cam1,distCoef_cam2
from cv2 import circle,destroyAllWindows,triangulatePoints,cvtColor,line,COLOR_GRAY2RGB,computeCorrespondEpilines,FONT_HERSHEY_SIMPLEX,putText
from myLib import orderCenterCoord,estimateFundMatrix_8norm,decomposeEssentialMat,myProjectionPoints,isCollinear,isEqual,getSignal
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import CubicSpline
from itertools import permutations,combinations

def getTheClosest(coordNow, prev):
    # get data and new arrays
    centerCoord,prevCenterCoord = np.copy(coordNow).reshape(-1,2),np.copy(prev).reshape(-1,2)
    newOrder,nMarkers = np.ones(centerCoord.shape[0],dtype=np.int8)*-1,centerCoord.shape[0]
    for i in range(nMarkers): # interate per previous coordinate
        if newOrder[i] == -1:
            # get the previous coordinate
            pt = prevCenterCoord[i]      
            # compute distance to actual centroids
            dist = np.linalg.norm(centerCoord-pt,axis=1)
            retNow = dist < 5
            # if there is ony one blob tagged as closest
            if np.sum(retNow) <= 1: newOrder[i] = np.argmin(dist)
            else: # if there are ambiguous blobs
                # get all combinations of prev-actual blobs
                allPermuationsOf4,allDists = np.array(list(permutations(list(range(0,4))))),[]
                for idx in allPermuationsOf4:
                    newPts,dist = centerCoord[idx],0
                    for k in range(nMarkers): 
                        aux= np.linalg.norm(prevCenterCoord[k]-newPts[k])
                        dist+=aux
                    allDists.append(dist)
                minDist = np.argmin(allDists)
                choosenIdx = allPermuationsOf4[minDist]
                return choosenIdx
    return newOrder

def myInterpolate(coord,ts,step):
    if not len(ts): return [],[]
    lowBound,highBound = math.ceil(ts[0]/step),math.floor(ts[-1]/step)
    tNew = np.linspace(lowBound,highBound,int((highBound-lowBound))+1,dtype=np.uint16)
    ff = CubicSpline(ts,coord,axis=0)
    return ff(tNew*step),tNew

def getEpilineCoef(pts,F):
    [a,b,c]=np.matmul(F,np.hstack((pts,1))) #ax+by+c=0
    return [a,b,c]/(np.sqrt(pow(a,2)+pow(b,2)))

def getDistance2Line(lines,pts):
    pts,out,lines = np.copy(pts).reshape(-1,2),[],np.copy(lines).reshape(-1,3)
    for [a,b,c] in lines:
        for [x,y] in pts: out.append(abs(a*x+b*y+c)/np.sqrt(np.sqrt(pow(a,2)+pow(b,2))))
    return np.array(out)<5,np.array(out) 

def drawlines(img1,img2,lines,pts1,pts2):    
    r,c = img1.shape
    img1 = cvtColor(img1.astype('float32'),COLOR_GRAY2RGB)
    img2 = cvtColor(img2.astype('float32'),COLOR_GRAY2RGB)
    listColors = [(0,0,255),(0,255,0),(255,0,0),(255,0,255)]
    i = 0
    for r,pt1,pt2 in zip(lines,pts1,pts2):
        color = listColors[i]
        x0,y0 = map(int, [0, -r[2]/r[1] ])
        x1,y1 = map(int, [c, -(r[2]+r[0]*c)/r[1] ])
        img1 = line(img1, (x0,y0), (x1,y1), color,1)
        img1 = circle(img1,tuple(pt1),5,color,-1)
        putText(img1,str(i),tuple(pt1-20),FONT_HERSHEY_SIMPLEX,0.5,color,2) 
        img2 = circle(img2,tuple(pt2),5,color,-1)
        putText(img2,str(i),tuple(pt2-20),FONT_HERSHEY_SIMPLEX,0.5,color,2) 
        i+=1
    return img1,img2

class myServer(object):
    def __init__(self,numberCameras,triggerTime,recTime,FPS,verbose,output,save,firstIP):
        ##########################################
        # PLEASE CHANGE JUST THE VARIABLES BELOW #
        ##########################################
        self.numberCameras,self.triggerTime,self.recTime,self.step,self.out,self.save,self.verbose,self.firstIP = numberCameras,triggerTime,recTime,1/FPS,output,save,verbose,firstIP
        self.cameraMat = np.array([cameraMatrix_cam1,cameraMatrix_cam2])
        # do not change below this line, socket variables
        self.nImages,self.imageSize = int(self.recTime/self.step),[]
        print('[INFO] creating server')
        self.bufferSize,self.server_socket = 1024,socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        self.server_socket.bind(('0.0.0.0',8888))
        # internal variables
        self.addDic,self.myIPs,self.P= {},[],[]
        for i in range(self.numberCameras):  self.P.append([]) 
        self.R,self.t = np.genfromtxt('R.csv', delimiter=','),np.genfromtxt('t.csv', delimiter=',').reshape(-1,3)
        self.lamb,self.F = np.genfromtxt('lamb.csv',delimiter=','),np.genfromtxt('F.csv', delimiter=',')
        self.R_plane,self.t_plane = np.genfromtxt('R_plane.csv', delimiter=','),np.genfromtxt('t_plane.csv', delimiter=',')
        
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
        # set projection matrices
        print('[INFO] projection matrices set')
        self.P[0],self.P[1] = np.hstack((self.cameraMat[0],[[0.],[0.],[0.]])),np.matmul(self.cameraMat[1], np.hstack((self.R,self.t.T)))
        # SEND TRIGGER 
        print('[INFO] all clients connected')
        self.triggerTime += time.time()
        for i in range(self.numberCameras): self.server_socket.sendto((str(self.triggerTime)+' '+str(self.recTime)).encode(),tuple(self.myIPs[i]))
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
                print('conversion for intrinsics matrix not known')
                return False,camIntris
            print('[INFO] intrisics known')
            return True,camIntris
        else:
            print('out of proportion of the camera mode')
            return False,camIntris

    def getOrderPerEpiline(self,coord1,coord2,nMarkers,verbose = 0):
        # get data
        pts1,pts2,orderSecondFrame = np.copy(coord1).reshape(-1,2),np.copy(coord2).reshape(-1,2),np.ones(nMarkers,dtype=np.int8)*-1
        epilines = np.zeros((nMarkers,3))
        choosenIdx,allDists = [],[]
        for i in range(nMarkers):
            epilines[i] = getEpilineCoef(pts1[i],self.F)        
        allPermuationsOf4 = np.array(list(permutations(list(range(0,nMarkers)))))
        # get all possible distances between point/line
        for idx2 in allPermuationsOf4:
            newPts2,dist = pts2[idx2],0
            for k in range(nMarkers): 
                _,aux= getDistance2Line(epilines[k],newPts2[k])
                dist+=aux
            allDists.append(dist)
        # get minimum distance
        minDist = min(allDists)
        # get all idx with 1 pixels distance from the mininum distance combination
        choosenIdx = allPermuationsOf4[np.where(allDists<=minDist+1)[0]]
        # id there are more than 1 combination possible, find ambiguous blobs
        if len(choosenIdx)>1:
            # initiate variables
            allCombinationsOf2 = np.array(list(combinations(list(range(0,len(choosenIdx))),2)))
            mask = np.ones(nMarkers,dtype=np.bool)
            # get common idx from all ambiguous combinations
            for idx in allCombinationsOf2:
                nowMask = np.equal(choosenIdx[idx[0]],choosenIdx[idx[1]])
                mask*=nowMask
            # invert mask to find the blobs that differs in each image
            mask = np.invert(mask)
            idxAmbiguous1,idxAmbiguous2 = np.where(mask)[0],choosenIdx[0][mask]
            collinearPts1,collinearPts2 = pts1[idxAmbiguous1],pts2[idxAmbiguous2]
            # sort per X
            if np.all(np.diff(np.sort(collinearPts2[:,0]))>2) and np.all(np.diff(np.sort(collinearPts1[:,0]))>2):
                order1,order2 = np.argsort(collinearPts1[:,0]),np.argsort(collinearPts2[:,0])
                idx1 = np.hstack((collinearPts1,idxAmbiguous1.reshape(idxAmbiguous1.shape[0],-1)))[order1,-1].T
                idx2 = np.hstack((collinearPts2,idxAmbiguous2.reshape(idxAmbiguous2.shape[0],-1)))[order2,-1].T
            else: # sort per Y
                order1,order2 = np.argsort(collinearPts1[:,1]),np.argsort(collinearPts2[:,1])
                idx1 = np.hstack((collinearPts1,idxAmbiguous1.reshape(idxAmbiguous1.shape[0],-1)))[order1,-1].T
                idx2 = np.hstack((collinearPts2,idxAmbiguous2.reshape(idxAmbiguous2.shape[0],-1)))[order2,-1].T
            orderSecondFrame = choosenIdx[0]
            orderSecondFrame[idx1.astype(int)] = idx2.astype(int)
        else: orderSecondFrame = choosenIdx[0]
    
        ## verbose
        if verbose:
            pts2 = np.copy(pts2[orderSecondFrame])
            img1,img2 = np.ones((720,960))*255,np.ones((720,960))*255
            lines1 = computeCorrespondEpilines(np.int32(pts2).reshape(-1,1,2), 2,self.F)
            lines1 = lines1.reshape(-1,3)
            img5,_ = drawlines(img1,img2,lines1,np.int32(pts1),np.int32(pts2))
            lines2 = computeCorrespondEpilines(np.int32(pts1).reshape(-1,1,2), 1,self.F)
            lines2 = lines2.reshape(-1,3)
            img3,_ = drawlines(img2,img1,lines2,np.int32(pts2),np.int32(pts1))
            plt.figure(figsize=(20, 16),dpi=100)
            plt.subplot(121),plt.imshow(img5)
            plt.subplot(122),plt.imshow(img3)
            plt.show()
        return orderSecondFrame

    # COLLECT POINTS FROM CLIENTS, ORDER AND TRIGGER INTERPOLATION
    def collect(self):
        # internal variables
        print('[INFO] waiting capture')
        capture = np.ones(self.numberCameras,dtype=np.bool)
        counter,lastTime,lastImgNumber = np.zeros(2,dtype=np.int32),np.zeros(2,dtype=np.int32),np.zeros(2,dtype=np.int32)
        missed,invalid,needsOrder = np.zeros(2,dtype=np.int32),np.zeros(2,dtype=np.int32),True
        dfSave,dfOrig,points3d,intervals,lastTnew = [],[],[],[],[]
        nPrevious,warmUp = 3,10
        for i in range(self.numberCameras): 
            intervals.append([])
            lastTnew.append([])
            dfOrig.append([])    
        dfInterp = np.zeros((self.nImages,self.numberCameras*8+1))
        dfInterp[:,-1] = np.linspace(0,self.recTime,self.nImages)  
        # capture loop
        try:
            while np.any(capture):
                #  receive message
                bytesPair = self.server_socket.recvfrom(self.bufferSize)
                message = np.frombuffer(bytesPair[0],dtype=np.float64)
                address,sizeMsg = bytesPair[1],len(message)
                idx = self.addDic[address[0]]
                if not (sizeMsg-1): capture[idx] = 0
                # if valid message
                if capture[idx]: # check if message is valid
                    if sizeMsg != 16: # if less than 4 blobs, discard
                        if self.verbose: print('[ERROR] '+str(int((sizeMsg-4)/3))+' markers were found')
                        missed[idx]+=1
                    else: 
                        msg = message[0:sizeMsg-4].reshape(-1,3)
                        coord,size = msg[:,0:2],msg[:,2].reshape(-1)
                        # if more than 4 blobs are found, get the four biggest
                        '''if sizeMsg > 16: 
                            orderAscDiameters = np.argsort(size)
                            coord = np.array([coord[orderAscDiameters[-1]],coord[orderAscDiameters[-2]],coord[orderAscDiameters[-3]],coord[orderAscDiameters[-4]]]).reshape(-1,2)
                        '''# store message parameters
                        a,b,time,imgNumber = message[-4],message[-3],message[-2],int(message[-1]) 
                        # undistort points
                        if address[0] == self.firstIP: undCoord = processCentroids_calib(coord,a,b,self.cameraMat[0],distCoef_cam1)
                        else: undCoord = processCentroids_calib(coord,a,b,self.cameraMat[1],distCoef_cam2)     
                        if self.save: dfSave.append(np.concatenate((undCoord.reshape(8),[time,imgNumber,idx])))
                        # if ts if not read corectly, discard
                        if counter[idx]:
                            if abs(time-lastTime[idx])>1e9: 
                                if self.verbose: print('[WARNING] time missmatch')
                                missed[idx]+=1
                                invalid[idx]+=1
                                continue
                        # check if sequence is valid
                        if imgNumber>lastImgNumber[idx]+1: invalid[idx] = imgNumber-lastImgNumber[idx]
                        # order markers per proximity and check collinearity
                        if not isEqual(undCoord,5) and not np.any(undCoord<0):     
                            if invalid[idx]>=10 or not counter[idx]: 
                                if self.verbose: print('reseting at', idx,counter[idx])
                                prev,needsOrder = [],True
                                intervals[idx].append(counter[idx])
                            else:
                                if not (counter[idx]-1): prev = np.array(dfOrig[idx][0:8]).reshape(-1,2)
                                else: prev = np.array(dfOrig[idx][-1,0:8]).reshape(-1,2)
                                newOrder = getTheClosest(undCoord.reshape(-1,2),prev.reshape(-1,2))
                                undCoord = np.copy(undCoord[newOrder])
                        else: 
                            if self.verbose: print('not collinear or equal centroids')
                            missed[idx]+=1
                            invalid[idx]+=1
                            continue
                        # update loop variables
                        lastTime[idx],lastImgNumber[idx],invalid[idx] = time,imgNumber,0    
                        if not counter[idx]: dfOrig[idx] = np.hstack((undCoord.reshape(8),time))
                        else: dfOrig[idx] = np.vstack((dfOrig[idx],np.hstack((undCoord.reshape(8),time))))
                        counter[idx]+=1
                        # interpolate
                        if np.all(counter):
                            # if valid interval has not been ordered
                            if needsOrder:
                                # if there are enough points at the actual interval
                                if np.all(counter-[intervals[0][-1],intervals[1][-1]]>=nPrevious):
                                    if self.verbose: print('in storage',counter-[intervals[0][-1],intervals[1][-1]])
                                    # see if there are intersection between arrays
                                    ts1,ts2 = dfOrig[0][intervals[0][-1]:counter[0],8]/1e6,dfOrig[1][intervals[1][-1]:counter[1],8]/1e6
                                    validIdx1 = [k for k in range(0,len(ts1)) if max(ts1[0], ts2[0])-0.01<=ts1[k]<=min(ts1[-1], ts2[-1])+0.01]
                                    validIdx2 = [k for k in range(0,len(ts2)) if max(ts1[0], ts2[0])-0.01<=ts2[k]<=min(ts1[-1], ts2[-1])+0.01]
                                    # if there is intersection, get order between the first intersection point
                                    if len(validIdx1) and len(validIdx2):
                                        # get intersection data
                                        ts1,ts2 = np.copy(ts1[validIdx1]),np.copy(ts2[validIdx2])
                                        if ts1.shape[0]<2 or ts2.shape[0]<2: continue
                                        coord1,coord2 = dfOrig[0][intervals[0][-1]:counter[0],0:8],dfOrig[1][intervals[1][-1]:counter[1],0:8]
                                        coord1,coord2 = np.copy(coord1[validIdx1]),np.copy(coord2[validIdx2])
                                        # get interpolated data
                                        interp1,tNew1 = myInterpolate(coord1,ts1,self.step)
                                        interp2,tNew2 = myInterpolate(coord2,ts2,self.step)
                                        if not len(interp1) or not len(interp2): continue
                                        # get common idx
                                        interpolateIdx1,interpolateIdx2 = np.argmax(np.in1d(tNew1, tNew2)),np.argmax(np.in1d(tNew2, tNew1))
                                        orderSecondFrame = self.getOrderPerEpiline(interp1[interpolateIdx1],interp2[interpolateIdx2],4,0)
                                        # get interval to rearrange
                                        idxInvalid = np.argmin(counter-[intervals[0][-1],intervals[1][-1]])
                                        beg = [k for k in range(intervals[1][-1],counter[1]) if dfOrig[idxInvalid][intervals[idxInvalid][-1],8]-0.01<=dfOrig[1][k,8]][0]
                                        if self.verbose:print('rearranging interval',[beg,counter[1]], 'to', orderSecondFrame)
                                        # flip blobs
                                        for k in range(beg,counter[1]):
                                            dfOrig[1][k,0:8] = np.copy(dfOrig[1][k,0:8].reshape(-1,2)[orderSecondFrame].reshape(-8))
                                        needsOrder=False
                            # if there is enough to interpolate
                            if np.all(counter-[intervals[0][-1],intervals[1][-1]]>=warmUp): 
                                # get data
                                coord,ts = dfOrig[idx][(counter[idx]-warmUp):counter[idx],0:8],dfOrig[idx][(counter[idx]-warmUp):counter[idx],8]/1e6
                                if not len(ts): continue
                                # find bound to interpolate
                                lowBound,highBound = math.ceil(ts[0]/self.step),math.floor(ts[-1]/self.step)
                                # do not repeat interpolation
                                if lastTnew[idx]:
                                    if lowBound<lastTnew[idx]: lowBound = lastTnew[idx]+1
                                tNew = np.linspace(lowBound,highBound,int((highBound-lowBound))+1,dtype=np.uint16)
                                # interpolate
                                ff = CubicSpline(ts,coord,axis=0)
                                dfInterp[tNew,int(idx*8):int(idx*8+8)] = ff(tNew*self.step)
                                lastTnew[idx],otherIdx = tNew[-1],int(not idx)
                                # for each new interpolated data, check if line is complete
                                for k in tNew:
                                    # if line is full, triangulate points
                                    if np.all(dfInterp[k,int(otherIdx*8):int(otherIdx*8+8)]):
                                        # get projections points
                                        centroids1,centroids2 = dfInterp[k,0:8].reshape(-1,2),dfInterp[k,8:16].reshape(-1,2)
                                        projPt1,projPt2 = myProjectionPoints(np.array(centroids1)),myProjectionPoints(np.array(centroids2))
                                        # triangulate points
                                        points4d = triangulatePoints(self.P[0].astype(float),self.P[1].astype(float),projPt1.astype(float),projPt2.astype(float))
                                        myPoints3d = (points4d[:3, :]/points4d[3, :]).T
                                        if myPoints3d[0, 2] < 0: myPoints3d = -myPoints3d
                                        # add to array
                                        if not len(points3d): points3d = np.copy(myPoints3d)
                                        else: points3d = np.vstack((points3d,myPoints3d))
                                            
        finally:        
            # close everything
            self.server_socket.close()
            destroyAllWindows()
            # save results
            if self.save: np.savetxt('camTest.csv', np.array(dfSave), delimiter=',')
            # plot results
            points3dNew = points3d*self.lamb/100
            fig = plt.figure(figsize=(8, 8))
            ax = plt.axes(projection='3d')
            ax.set_xlim(-1, 4)
            ax.set_zlim(-3, 0)
            ax.set_ylim(0, 4)
            ax.set_xlabel('X')
            ax.set_ylabel('Z')
            ax.set_zlabel('Y')
            cmhot = plt.get_cmap("jet")
            # plot first camera
            scale = 0.8
            cam1Pts = np.array([[0,self.t_plane,0],
                                [scale, 0, 0],
                                [0, scale, 0],
                                [0, 0, scale]])
            cam1PtsNew = np.matmul(self.R_plane,cam1Pts.T).T
            [cam1Root,x,y,z]=cam1PtsNew
            zDisplacement = cam1PtsNew[0,2]
            cam1Root+=[0,0,-zDisplacement]
            x,y,z=x/np.linalg.norm(x)*scale,y/np.linalg.norm(y)*scale,z/np.linalg.norm(z)*scale
            ax.quiver(cam1Root[0],cam1Root[2],cam1Root[1],x[0],x[2],x[1], arrow_length_ratio=0.1, edgecolors="r")
            ax.quiver(cam1Root[0],cam1Root[2],cam1Root[1],y[0],y[2],y[1], arrow_length_ratio=0.1, edgecolors="b")
            ax.quiver(cam1Root[0],cam1Root[2],cam1Root[1],z[0],z[2],z[1], arrow_length_ratio=0.1, edgecolors="g")
            ax.scatter(cam1Root[0],cam1Root[2],cam1Root[1], s=50, edgecolor="fuchsia", facecolor="plum", linewidth=2)
            # plot second camera
            x,y,z = np.array([scale, 0, 0]), np.array([0, scale, 0]),np.array([0, 0, scale])
            x,y,z = np.matmul(self.R.T, x),np.matmul(self.R.T, y),np.matmul(self.R.T, z)
            t_aux = np.matmul(-self.t, self.R)[0]*self.lamb/100+[0,self.t_plane,0]
            cam2Pts = np.array([t_aux.T,x,y,z])
            cam2PtsNew = np.matmul(self.R_plane,cam2Pts.T).T
            [cam2Root,x,y,z]=cam2PtsNew
            cam2Root+=[0,0,-zDisplacement]
            x,y,z=x/np.linalg.norm(x)*scale,y/np.linalg.norm(y)*scale,z/np.linalg.norm(z)*scale
            ax.quiver(cam2Root[0],cam2Root[2],cam2Root[1],x[0],x[2],x[1], arrow_length_ratio=0.1, edgecolors="r")
            ax.quiver(cam2Root[0],cam2Root[2],cam2Root[1],y[0],y[2],y[1], arrow_length_ratio=0.1, edgecolors="b")
            ax.quiver(cam2Root[0],cam2Root[2],cam2Root[1],z[0],z[2],z[1], arrow_length_ratio=0.1, edgecolors="g")
            ax.scatter(cam2Root[0],cam2Root[2],cam2Root[1], s=50, edgecolor="darkorange", facecolor="gold", linewidth=2)
            # new plane
            points3dNew = np.matmul(self.R_plane,points3dNew.T+np.array([[0],[self.t_plane],[0]])).T
            ax.scatter(points3dNew[:, 0], points3dNew[:, 2]-zDisplacement, points3dNew[:, 1], c=points3dNew[:, 2], cmap=cmhot)
            # axis setup
            cmhot = plt.get_cmap("jet")
            ax.view_init(elev=1, azim=-50) 
            plt.gca().invert_zaxis()
            ax.get_proj = lambda: np.dot(Axes3D.get_proj(ax), np.diag([1., 1., .3, 1.]))
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
parser.add_argument('-ip',type=str,help='IP from first camera (default: 192.168.0.102)',default='192.168.0.102')
args = parser.parse_args()

myServer_ = myServer(args.n,args.trig,args.rec,args.fps,args.verbose,args.out,args.save,args.ip)
myServer_.connect()
myServer_.collect()