# IMPORTS >>> DO NOT CHANGE <<<
import warnings
warnings.filterwarnings("ignore")
import socket,time,math,argparse
import numpy as np
from constants import cameraMat,distCoef
from cv2 import destroyAllWindows,triangulatePoints
from myLib import myProjectionPoints,isEqual4,processCentroids,createNeedsOrder,activateNeedsOrder,getTheClosest,myInterpolate,getOrderPerEpiline,popNeedsOrder,getOtherValidIdx
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import CubicSpline

class myServer(object):
    def __init__(self,numberCameras,triggerTime,recTime,FPS,verbose,save,ipList):
        ##########################################
        # PLEASE DONT CHANGE THE VARIABLES BELOW #
        ##########################################
        self.numberCameras,self.triggerTime,self.recTime,self.step,self.save,self.verbose,self.ipList = numberCameras,triggerTime,recTime,1/FPS,save,verbose,np.array(ipList.split(','))
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
        counter,lastTime,lastImgNumber = np.zeros(self.numberCameras,dtype=np.int32),np.zeros(self.numberCameras,dtype=np.int32),np.zeros(self.numberCameras,dtype=np.int32)
        missed,invalid,swap = np.zeros(self.numberCameras,dtype=np.int32),np.zeros(self.numberCameras,dtype=np.int32),np.zeros(self.numberCameras,dtype=np.int32)
        dfSave,dfOrig,points3d,intervals,lastTnew ,allPoints3d= [],[],[],[],[],[]
        nPrevious,warmUp = 3,10 # warm up > n previous
        needsOrder = createNeedsOrder(self.numberCameras,relateLast2First=0)
        for i in range(self.numberCameras): 
            intervals.append([])
            lastTnew.append([])
            dfOrig.append([])    
        dfInterp = np.zeros((self.nImages,self.numberCameras*8+2))
        dfInterp[:,-2] = np.arange(0,self.recTime,self.step)
        dfInterp[:,-1] = np.zeros_like(dfInterp[:,-1],dtype=np.bool8)
        dfTriang = np.zeros((int(self.recTime/self.step),4*4+1))
        dfTriang[:,-1] = np.arange(0,self.recTime,self.step)
        # import R,T and lambda
        rotation = np.genfromtxt('data/R.csv', delimiter=',').reshape(-1,3,3)
        translation = np.genfromtxt('data/t.csv', delimiter=',').reshape(-1,1,3)
        projMat = np.genfromtxt('data/projMat.csv', delimiter=',').reshape(-1,4,4)
        scale,FMatrix = np.genfromtxt('data/lamb.csv', delimiter=','), np.genfromtxt('data/F.csv', delimiter=',').reshape(-1,3,3)
        P_plane = np.genfromtxt('data/P_plane.csv', delimiter=',').reshape(-1,4)
        [d,b,zDisplacement] = np.genfromtxt('data/groundData.csv', delimiter=',').reshape(3)
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
                        a,b,timeNow,imgNumber = message[-4],message[-3],message[-2],int(message[-1]) 
                        # undistort points
                        undCoord = processCentroids(coord,a,b,self.cameraMat[idx],distCoef[idx])
                        if self.save: dfSave.append(np.concatenate((undCoord.reshape(8),[timeNow,imgNumber,idx])))
                        # if ts if not read corectly, discard
                        if counter[idx]:
                            if abs(timeNow-lastTime[idx])>1e9: 
                                if self.verbose: print('time missmatch')
                                missed[idx]+=1
                                invalid[idx]+=1
                                continue
                        # check if sequence is valid
                        if imgNumber>lastImgNumber[idx]+1: invalid[idx] = imgNumber-lastImgNumber[idx]
                        # check occlusion and order markers per proximity
                        if not isEqual4(undCoord,5) and not np.any(undCoord<0):     
                            if invalid[idx]>=10 or not counter[idx]: 
                                if self.verbose: print('reseting at camera', idx,', counter',counter[idx],',',timeNow/1e6,'s')
                                prev,needsOrder = [],activateNeedsOrder(self.numberCameras,idx,needsOrder,relateLast2First=0)
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
                        lastTime[idx],lastImgNumber[idx],invalid[idx] = timeNow,imgNumber,0    
                        if not counter[idx]: dfOrig[idx] = np.hstack((undCoord.reshape(8),timeNow))
                        else: dfOrig[idx] = np.vstack((dfOrig[idx],np.hstack((undCoord.reshape(8),timeNow))))
                        counter[idx]+=1
                        # interpolate
                        if needsOrder[str(idx)].size:
                            # get is there are enough points at the valid interval
                            for otherIdx in needsOrder[str(idx)]:
                                if not len(intervals[otherIdx]): continue
                                myCounter,myIntervals = np.array([counter[idx],counter[otherIdx]]),np.array([intervals[idx][-1],intervals[otherIdx][-1]])            
                                if np.all(myCounter-myIntervals>=nPrevious):
                                    # see if there are intersection between arrays
                                    ts1,ts2 = dfOrig[idx][intervals[idx][-1]:counter[idx],8]/1e6,dfOrig[otherIdx][intervals[otherIdx][-1]:counter[otherIdx],8]/1e6
                                    validIdx1 = [k for k in range(0,len(ts1)) if max(ts1[0], ts2[0])-0.01<=ts1[k]<=min(ts1[-1], ts2[-1])+0.01]
                                    validIdx2 = [k for k in range(0,len(ts2)) if max(ts1[0], ts2[0])-0.01<=ts2[k]<=min(ts1[-1], ts2[-1])+0.01]
                                    # if there is intersection, get order
                                    if len(validIdx1) and len(validIdx2):
                                        ts1,ts2 = np.copy(ts1[validIdx1]),np.copy(ts2[validIdx2])
                                        if ts1.shape[0]<2 or ts2.shape[0]<2: continue
                                        coord1,coord2 = dfOrig[idx][intervals[idx][-1]:counter[idx],0:8],dfOrig[otherIdx][intervals[otherIdx][-1]:counter[otherIdx],0:8]
                                        coord1,coord2 = np.copy(coord1[validIdx1]),np.copy(coord2[validIdx2])
                                        # get interpolated data
                                        interp1,tNew1 = myInterpolate(coord1,ts1,self.step)
                                        interp2,tNew2 = myInterpolate(coord2,ts2,self.step)
                                        if not len(interp1) or not len(interp2): continue
                                        # get min and max idx
                                        minIdx,maxIdx = min(idx,otherIdx),max(idx,otherIdx)
                                        # get common idx
                                        F = FMatrix[minIdx]
                                        interpolateIdx1,interpolateIdx2 = np.argmax(np.in1d(tNew1, tNew2)),np.argmax(np.in1d(tNew2, tNew1))
                                        # order per epipolar line
                                        if idx < otherIdx: orderSecondFrame,ret = getOrderPerEpiline(interp1[-1],interp2[-1],4,F,0,1)
                                        else: orderSecondFrame,ret = getOrderPerEpiline(interp2[-1],interp1[-1],4,F,0,1)
                                        if not ret: 
                                            if self.verbose: print(minIdx,maxIdx,'could not rearange at',tNew2[-1]*self.step,'s')
                                            continue
                                        # get interval to rearrange
                                        if self.verbose: print(minIdx,maxIdx,'rearanging interval',[intervals[maxIdx][-1],counter[maxIdx]], 'to', orderSecondFrame, 'at',tNew2[-1]*self.step,'s')
                                        # flip blobs
                                        for k in range(intervals[maxIdx][-1],counter[maxIdx]):
                                            dfOrig[maxIdx][k,0:8] = np.copy(dfOrig[maxIdx][k,0:8].reshape(-1,2)[orderSecondFrame].reshape(-8))
                                        # change ordering boolean
                                        needsOrder=popNeedsOrder(idx,otherIdx,needsOrder)
                                        for k in range(maxIdx+1,self.numberCameras):
                                            needsOrder=activateNeedsOrder(self.numberCameras,k,needsOrder,relateLast2First=0) 
                        if (counter[idx]-intervals[idx][-1])>=warmUp: 
                            # get data to interpolate
                            coord,ts = dfOrig[idx][(counter[idx]-warmUp):counter[idx],0:8],dfOrig[idx][(counter[idx]-warmUp):counter[idx],8]/1e6
                            if not len(ts): continue
                            lowBound,highBound = math.ceil(ts[0]/self.step),math.floor(ts[-1]/self.step)
                            # avoid repeating intepolation
                            if lastTnew[idx]:
                                if lowBound<lastTnew[idx]: lowBound = lastTnew[idx]+1
                            # interpolate
                            tNew = np.linspace(lowBound,highBound,int((highBound-lowBound))+1,dtype=np.uint16)
                            ff = CubicSpline(ts,coord,axis=0)
                            # save data
                            dfInterp[tNew,int(idx*8):int(idx*8+8)] = ff(tNew*self.step)
                            lastTnew[idx] = tNew[-1]
                            # compare if there is another camera available
                            for k in tNew:
                                if dfInterp[k,-1]: continue
                                otherIdx = getOtherValidIdx(dfInterp[k,:],4,idx)      
                                #print(k,otherIdx)
                                if not len(otherIdx) or otherIdx in needsOrder[str(idx)]: continue      
                                # get index from the other camera
                                dfInterp[k,-1],otherIdx = True,otherIdx[0]
                                # comparing the indexes to set the projection matrices
                                minIdx,maxIdx = min(idx,otherIdx),max(idx,otherIdx)
                                # getting the data
                                pts1,pts2 = dfInterp[k,minIdx*8:(minIdx+1)*8].reshape(-1,2),dfInterp[k,maxIdx*8:(maxIdx+1)*8].reshape(-1,2)
                                R,t,lamb = rotation[maxIdx],translation[maxIdx].reshape(-1,3),scale[maxIdx]
                                P1,P2 = np.hstack((cameraMat[minIdx], [[0.], [0.], [0.]])),np.matmul(cameraMat[maxIdx], np.hstack((R, t.T)))
                                projPt1,projPt2 = myProjectionPoints(np.array(pts1)),myProjectionPoints(np.array(pts2))
                                # triangulate
                                points4d = triangulatePoints(P1.astype(float),P2.astype(float),projPt1.astype(float),projPt2.astype(float))
                                points3d = (points4d[:3, :]/points4d[3, :]).T
                                if points3d[0, 2] < 0: points3d = -points3d  
                                # project in scale regarding the minimum index camera
                                points3d = np.hstack((points3d*lamb/100,np.ones((points3d.shape[0],1)))).T
                                points3d = np.matmul(projMat[minIdx],points3d).T
                                # rotate to ground plane
                                points3d+= [0,d/b,0,0]
                                points3d = np.matmul(P_plane,points3d.T).T
                                points3d+= [0,0,-zDisplacement,0]
                                points3d = points3d.T
                                # apply correction matrix
                                #if minIdx: points3d = np.matmul(correcMat[minIdx-1],points3d)
                                points3d = points3d.T
                                # save to array
                                dfTriang[k,0:16] = np.copy(points3d.ravel())
                                if not len(allPoints3d):
                                    allPoints3d = np.copy(points3d)
                                else: allPoints3d = np.vstack((allPoints3d,points3d)) 
                                #print(minIdx,maxIdx,round(k*0.01,2),points3d[0])
                                            
        finally:        
            # close everything
            self.server_socket.close()
            destroyAllWindows()
            # save results
            if self.save: np.savetxt('data/camTest.csv', np.array(dfSave), delimiter=',')
            # just comprising dataset if wanted to plot
            emptyLines = np.unique([i for i in range(0,dfInterp.shape[0]) if not dfInterp[i][-1]])
            dfInterp = np.delete(dfInterp,emptyLines,axis=0)
            dfTriang = np.delete(dfTriang,emptyLines,axis=0)
            print('[INFO] found ' +str(dfTriang.shape[0])+ ' interpolated pics')
            allPoints3d = allPoints3d.T
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

            # plot each camera translated and rotated to meet the ground plane
            for j in range(self.numberCameras):
                o = np.matmul(projMat[j],[[0.],[0],[0.],[1]]).ravel()
                o+= [0,+d/b,0,0]
                o = np.matmul(P_plane,o).ravel()
                o+= [0,0,-zDisplacement,0]
                x,y,z= np.array([1, 0, 0, 0]), np.array([0, 1, 0, 0]),np.array([0, 0, 1, 0])
                x,y,z = np.matmul(projMat[j],x),np.matmul(projMat[j],y),np.matmul(projMat[j],z)
                x,y,z = np.matmul(P_plane,x),np.matmul(P_plane,y),np.matmul(P_plane,z)
                ax.quiver(o[0], o[2], o[1], x[0], x[2], x[1], arrow_length_ratio=0.1, edgecolors="r", label='X axis')
                ax.quiver(o[0], o[2], o[1], y[0], y[2], y[1], arrow_length_ratio=0.1, edgecolors="b", label='Y axis')
                ax.quiver(o[0], o[2], o[1], z[0], z[2], z[1], arrow_length_ratio=0.1, edgecolors="g", label='Z axis')
                ax.scatter(o[0], o[2], o[1], s=50, edgecolor=colours[j][0], facecolor=colours[j][1], linewidth=2,  label = 'Camera '+str(j))

            # plot points after correction matrix
            ax.scatter(allPoints3d[0], allPoints3d[2], allPoints3d[1], s=50, c=allPoints3d[2], cmap=cmhot, label= 'Markers')

            # axis setup and plot variables
            handles, labels = plt.gca().get_legend_handles_labels()
            by_label = dict(zip(labels, handles))
            plt.legend(by_label.values(), by_label.keys(),ncol=3,loc ='center',edgecolor='silver', bbox_to_anchor=(0.5, 0.8))
            plt.draw()
            plt.show()
            
            
# parser for command line
parser = argparse.ArgumentParser(description='''Server for the MoCap system at the Erobotica lab of UFCG.
                                                \nPlease use it together with the corresponding client script.''',add_help=False)
parser.add_argument('-n',type=int,help='number of cameras (default: 2)',default=2)
parser.add_argument('-trig',type=int,help='trigger time in seconds (default: 10)',default=10)
parser.add_argument('-rec',type=int,help='recording time in seconds (default: 30)',default=30)
parser.add_argument('-fps',type=int,help='interpolation fps (default: 100FPS)',default=100)
parser.add_argument('--verbose',help='show ordering and interpolation verbosity (default: off)',default=False, action='store_true')
parser.add_argument('--help', action='help', default=argparse.SUPPRESS, help='show this help message and exit.')
parser.add_argument('-save',help='save received packages to CSV (default: off)',default=False, action='store_true')
parser.add_argument('-ip',type=str,help='List of IPs of each camera, in order',default='')
args = parser.parse_args()

myServer_ = myServer(args.n,args.trig,args.rec,args.fps,args.verbose,args.save,args.ip)
myServer_.connect()
myServer_.collect()