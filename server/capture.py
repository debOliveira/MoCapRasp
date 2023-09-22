# IMPORTS >>> DO NOT CHANGE <<<
import warnings
warnings.filterwarnings("ignore")
import socket,time,math,argparse
import numpy as np
from scipy.interpolate import CubicSpline
from cv2 import destroyAllWindows,triangulatePoints

from mcr.math import interpolate
from mcr.cameras import projectionPoints, getOtherValidIdx
from mcr.markers import occlusion, processCentroids, getOrderPerEpiline, createNeedsOrder, activateNeedsOrder, getTheClosest, getOrderPerEpiline, popNeedsOrder
from mcr.plot import plotArena
from mcr.constants import cameraMat, distCoef

class mcrServer(object):
    def __init__(self,nCameras,nMarkers,triggerTime,recTime,FPS,verbose,save):
        # VARIABLES >>> DO NOT CHANGE <<<
        self.nCameras = nCameras
        self.nMarkers = nMarkers
        self.triggerTime = triggerTime
        self.recTime = recTime
        self.FPS = FPS
        self.step = 1 / FPS
        self.verbose = verbose
        self.save = save

        self.ipList = []

        # IP lookup from hostname
        try:
            self.ipList = [socket.gethostbyname(f'cam{ID}.local') for ID in range(nCameras)]
        except socket.gaierror as e:
            print('[ERROR] number of cameras do not match the number of IPs found')
            exit()

        self.cameraMat = np.copy(cameraMat)
        self.distCoef = np.copy(distCoef)

        # Do not change below this line, socket variables
        self.nImages = int(self.recTime / self.step)
        self.imageSize = []
        
        for _ in range(self.nCameras): 
            self.imageSize.append([])
        
        print('[INFO] creating server')

        self.bufferSize = 1024
        self.server_socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM) 
        self.server_socket.bind(('0.0.0.0',8888))

    # Connects with clients
    def connect(self):
        print("[INFO] server running, waiting for clients")
        addedCams,ports,k=[],[],0
        while k != self.nCameras:
            # Collect adresses
            message,address = self.server_socket.recvfrom(self.bufferSize)

            # Check if it is in IP list
            if not len(np.where(self.ipList==address[0])[0]):
                print('[ERROR] IP '+address[0]+' not in the list')
                exit()

            # Get image size
            idx = np.where(self.ipList==address[0])[0][0]
            if idx in addedCams: continue
            self.imageSize[idx] = np.array(message.decode("utf-8").split(",")).astype(np.int)
            print('[INFO] camera '+str(idx)+' connected at '+str(address[0]))

            # Redo intrinsics
            ret,newCamMatrix=self.mcrIntrinsics(self.cameraMat[idx],self.imageSize[idx][0],self.imageSize[idx][1],self.imageSize[idx][2])
            if ret: self.cameraMat[idx]=np.copy(newCamMatrix)
            else: exit()
            k+=1
            addedCams.append(idx)
            ports.append(address)

        # Send trigger
        print('[INFO] all clients connected')
        self.triggerTime += time.time()
        for i in range(self.nCameras): self.server_socket.sendto((str(self.triggerTime)+' '+str(self.recTime)).encode(),tuple(ports[i]))
        print('[INFO] trigger sent')

    # New intrinsics
    def mcrIntrinsics(self,origMatrix,w,h,mode):
        camIntris = np.copy(origMatrix) # Copy to avoid registrer error

        # Check if image is at the vailable proportion
        if w/h==4/3 or w/h==16/9:
            if mode==4: # Only resize
                ratio = w/960
                camIntris[0][0],camIntris[0][2]=ratio*camIntris[0][0],ratio*camIntris[0][2]
                camIntris[1][1],camIntris[1][2]=ratio*camIntris[1][1],ratio*camIntris[1][2]
            
            elif mode==5: # Crop in X and resize
                ratio = 1640/960
                camIntris[0][0],camIntris[0][2]=ratio*camIntris[0][0],ratio*camIntris[0][2]
                camIntris[1][1],camIntris[1][2]=ratio*camIntris[1][1],ratio*camIntris[1][2]-155
                ratio = w/1640
                camIntris[0][0],camIntris[0][2]=ratio*camIntris[0][0],ratio*camIntris[0][2]
                camIntris[1][1],camIntris[1][2]=ratio*camIntris[1][1],ratio*camIntris[1][2]
            
            elif mode==6: # Crop in Y and X and resize
                ratio=1640/960
                camIntris[0][0],camIntris[0][2]=ratio*camIntris[0][0],ratio*camIntris[0][2]-180
                camIntris[1][1],camIntris[1][2]=ratio*camIntris[1][1],ratio*camIntris[1][2]-255
                ratio = w/1280
                camIntris[0][0],camIntris[0][2]=ratio*camIntris[0][0],ratio*camIntris[0][2]
                camIntris[1][1],camIntris[1][2]=ratio*camIntris[1][1],ratio*camIntris[1][2]
            
            elif mode==7: # Crop in Y and X and resize
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


    # Collect points from clients, order and trigger interpolation
    def collect(self):
        # Internal variables
        print('[INFO] waiting capture')
        capture = np.ones(self.nCameras,dtype=np.bool)
        counter,lastTime,lastImgNumber = np.zeros(self.nCameras,dtype=np.int32),np.zeros(self.nCameras,dtype=np.int32),np.zeros(self.nCameras,dtype=np.int32)

        missed = np.zeros(self.nCameras,dtype=np.int32)
        invalid = np.zeros(self.nCameras,dtype=np.int32)
        dfSave,dfOrig,points3d,intervals,lastTnew ,allPoints3d= [],[],[],[],[],[]

        nPrevious,warmUp = 3,10 # Warm up > nPrevious

        needsOrder = createNeedsOrder(self.nCameras,relateLast2First=0)

        for _ in range(self.nCameras): # For each camera
            intervals.append([]) # List for each camera
            lastTnew.append([]) # List for each camera
            dfOrig.append([]) # List for each camera

        # Each PTS will have a 2D marker coordinate for each camera + 2 additional columns
        dfInterp = np.zeros((self.nImages,self.nCameras*(2*self.nMarkers)+2)) # Last two columns are special

        dfInterp[:,-2] = np.arange(0,self.recTime,self.step) # Penultimate column is filled with the time stamps
        dfInterp[:,-1] = np.zeros_like(dfInterp[:,-1],dtype=np.bool8) # Last column is filled with bool zeros (false)

        # Each PTS will have...
        dfTriang = np.zeros((int(self.recTime/self.step),(4*self.nMarkers)+1)) # 4D coordinates of each marker + PTS
        dfTriang[:,-1] = np.arange(0,self.recTime,self.step) # Last column is filled with the PTSs

        # Import R,T and lambda
        rotation = np.genfromtxt('data/R.csv', delimiter=',').reshape(-1,3,3)
        translation = np.genfromtxt('data/t.csv', delimiter=',').reshape(-1,1,3)
        projMat = np.genfromtxt('data/projMat.csv', delimiter=',').reshape(-1,4,4)
        scale,FMatrix = np.genfromtxt('data/lamb.csv', delimiter=','), np.genfromtxt('data/F.csv', delimiter=',').reshape(-1,3,3)
        P_plane = np.genfromtxt('data/P_plane.csv', delimiter=',').reshape(-1,4)
        [d,b,h] = np.genfromtxt('data/groundData.csv', delimiter=',').reshape(3)

        # Capture loop
        try:
            while np.any(capture):
                # Receive message
                bytesPair = self.server_socket.recvfrom(self.bufferSize)
                message = np.frombuffer(bytesPair[0],dtype=np.float64)
                address,sizeMsg = bytesPair[1],len(message)
                idx = np.where(self.ipList==address[0])[0][0]
                if not (sizeMsg-1): capture[idx] = 0

                # If valid message
                if capture[idx]: # Check if message is valid
                    if sizeMsg < (3*self.nMarkers)+4: # If less than nMarker blobs are found, discard
                        if self.verbose: print('[ERROR] '+str(int((sizeMsg-4)/3))+' markers were found')
                        missed[idx]+=1
                    else: 
                        # Get data from the line
                        # Msg is: 2D Coordinates from each marker + blob area | xMin | yMin | PTS | image number
                        msg = message[0:sizeMsg-4].reshape(-1,3) # Separating blob info
                        coord,blobArea = msg[:,0:2], msg[:,2].reshape(-1) # Separating coordinates 

                        # If more than nMarker blobs are found, get the nMarker biggest
                        if sizeMsg > (3*self.nMarkers)+4:
                            orderAscDiameters = np.argsort(blobArea)[::-1] # Sorted by biggest to lowest diameter
                            coord = np.array([coord[biggestBlobs] for biggestBlobs in orderAscDiameters[:self.nMarkers]]).reshape(-1,2)
                    
                        # Store message parameters
                        a,b,timeNow,imgNumber = message[-4],message[-3],message[-2],int(message[-1])

                        # Undistort points
                        undCoord = processCentroids(coord,a,b,self.cameraMat[idx],distCoef[idx])
                        if self.save: dfSave.append(np.concatenate((undCoord.reshape(2*self.nMarkers),[timeNow,imgNumber,idx]))) 

                        # Time missmatch discarding
                        if counter[idx]:
                            if abs(timeNow-lastTime[idx])>1e9: 
                                if self.verbose: print('time missmatch')
                                missed[idx] += 1 # One more missed image for the camera
                                invalid[idx] += 1 # One more missed invalid for the camera
                                continue
                        
                        # Check if sequence is valid
                        if imgNumber>lastImgNumber[idx]+1: invalid[idx] = imgNumber-lastImgNumber[idx]

                        # Check occlusion and order markers per proximity
                        if not occlusion(undCoord,5) and not np.any(undCoord < 0): 
                            if invalid[idx] >= 10 or not counter[idx]: 
                                if self.verbose: print('reseting at camera', idx,', counter',counter[idx],',',timeNow/1e6,'s')
                                prev = []
                                needsOrder = activateNeedsOrder(self.nCameras,idx,needsOrder,relateLast2First=0)
                                intervals[idx].append(counter[idx])
                            else:
                                if not (counter[idx]-1): prev = np.array(dfOrig[idx][0:(2*self.nMarkers)]).reshape(-1,2) 
                                else: prev = np.array(dfOrig[idx][-1,0:(2*self.nMarkers)]).reshape(-1,2) 
                                newOrder = getTheClosest(undCoord.reshape(-1,2),prev.reshape(-1,2))
                                undCoord = np.copy(undCoord[newOrder])
                        else: 
                            if self.verbose: print('not collinear or equal centroids')
                            missed[idx] += 1 # One more missed image for the camera
                            invalid[idx] += 1 # One more missed invalid for the camera
                            continue

                        # Update loop variables
                        lastTime[idx],lastImgNumber[idx],invalid[idx] = timeNow,imgNumber,0    
                        if not counter[idx]: dfOrig[idx] = np.hstack((undCoord.reshape(2*self.nMarkers),timeNow)) 
                        else: dfOrig[idx] = np.vstack((dfOrig[idx],np.hstack((undCoord.reshape(2*self.nMarkers),timeNow)))) 
                        counter[idx] += 1

                        # Interpolate
                        if needsOrder[str(idx)].size:
                            # If there are enough points at the valid interval
                            for otherIdx in needsOrder[str(idx)]:
                                if not len(intervals[otherIdx]): continue
                                myCounter,myIntervals = np.array([counter[idx],counter[otherIdx]]),np.array([intervals[idx][-1],intervals[otherIdx][-1]])            
                                if np.all(myCounter-myIntervals>=nPrevious):
                                    # See if there are intersection between arrays
                                    ts1 = dfOrig[idx][intervals[idx][-1]:counter[idx],(2 * self.nMarkers)] / 1e6
                                    ts2 = dfOrig[otherIdx][intervals[otherIdx][-1]:counter[otherIdx],(2 * self.nMarkers)] / 1e6 

                                    maxFirstIntersection = max(ts1[0], ts2[0])
                                    minLastIntersection = min(ts1[-1], ts2[-1])
                                    
                                    validIdx1 = [k for k in range(0, len(ts1)) if maxFirstIntersection - 0.01 <= ts1[k] <= minLastIntersection + 0.01]
                                    validIdx2 = [k for k in range(0, len(ts2)) if maxFirstIntersection - 0.01 <= ts2[k] <= minLastIntersection + 0.01]
                                    
                                    # If there is intersection, get order
                                    if len(validIdx1) and len(validIdx2):

                                        ts1,ts2 = np.copy(ts1[validIdx1]),np.copy(ts2[validIdx2])

                                        if ts1.shape[0] < 2 or ts2.shape[0] < 2: continue

                                        coord1,coord2 = dfOrig[idx][intervals[idx][-1]:counter[idx],0:(2*self.nMarkers)],dfOrig[otherIdx][intervals[otherIdx][-1]:counter[otherIdx],0:(2*self.nMarkers)] 
                                        coord1,coord2 = np.copy(coord1[validIdx1]),np.copy(coord2[validIdx2])
                                        
                                        # Get interpolated data
                                        interp1,tNew1 = interpolate(coord1,ts1,self.step)
                                        interp2,tNew2 = interpolate(coord2,ts2,self.step)
                                        if not len(interp1) or not len(interp2): continue
                                        
                                        # Get min and max idx
                                        minIdx,maxIdx = min(idx,otherIdx),max(idx,otherIdx)

                                        # Get common idx
                                        F = FMatrix[minIdx]
                                        
                                        # Order per epipolar line
                                        if idx < otherIdx: orderSecondFrame,ret = getOrderPerEpiline(interp1[-1],interp2[-1],self.nMarkers,F,0,1) 
                                        else: orderSecondFrame,ret = getOrderPerEpiline(interp2[-1],interp1[-1],self.nMarkers,F,0,1) 
                                        if not ret: 
                                            if self.verbose: print(minIdx,maxIdx,'could not rearange at',tNew2[-1]*self.step,'s')
                                            continue
                                        
                                        # Get interval to rearrange
                                        if self.verbose: print(minIdx,maxIdx,'rearanging interval',[intervals[maxIdx][-1],counter[maxIdx]], 'to', orderSecondFrame, 'at',tNew2[-1]*self.step,'s')
                                        
                                        # Flip blobs
                                        for k in range(intervals[maxIdx][-1],counter[maxIdx]):
                                            dfOrig[maxIdx][k,0:(2*self.nMarkers)] = np.copy(dfOrig[maxIdx][k,0:(2*self.nMarkers)].reshape(-1,2)[orderSecondFrame].reshape(-(2*self.nMarkers))) 
                                        
                                        # Change ordering boolean
                                        needsOrder = popNeedsOrder(idx, otherIdx, needsOrder)
                                        for k in range(maxIdx+1,self.nCameras):
                                            needsOrder = activateNeedsOrder(self.nCameras,k,needsOrder,relateLast2First=0) 

                        if (counter[idx]-intervals[idx][-1])>=warmUp: 
                            # Get data to interpolate
                            coord,ts = dfOrig[idx][(counter[idx]-warmUp):counter[idx],0:(2*self.nMarkers)],dfOrig[idx][(counter[idx]-warmUp):counter[idx],(2*self.nMarkers)]/1e6 
                            if not len(ts): continue
                            lowBound,highBound = math.ceil(ts[0]/self.step),math.floor(ts[-1]/self.step)

                            # Avoid repeating intepolation
                            if lastTnew[idx]:
                                if lowBound<lastTnew[idx]: lowBound = lastTnew[idx]+1

                            # Interpolate
                            tNew = np.linspace(lowBound,highBound,int((highBound-lowBound))+1,dtype=np.uint16)
                            ff = CubicSpline(ts,coord,axis=0)

                            # Save data 
                            dfInterp[tNew,int(idx*(2*self.nMarkers)):int(idx*(2*self.nMarkers)+(2*self.nMarkers))] = ff(tNew*self.step) 
                            lastTnew[idx] = tNew[-1]

                            # Compare if there is another camera available
                            for k in tNew:
                                if dfInterp[k,-1]: continue
                                otherIdx = getOtherValidIdx(dfInterp[k,:],self.nMarkers,idx) 
                                if not len(otherIdx) or otherIdx in needsOrder[str(idx)]: continue      
                                
                                # Get index from the other camera
                                dfInterp[k,-1],otherIdx = True,otherIdx[0]

                                # Comparing the indexes to set the projection matrices
                                minIdx,maxIdx = min(idx,otherIdx),max(idx,otherIdx)
                                
                                # Getting the data
                                pts1,pts2 = dfInterp[k,minIdx*(2*self.nMarkers):(minIdx+1)*(2*self.nMarkers)].reshape(-1,2),dfInterp[k,maxIdx*(2*self.nMarkers):(maxIdx+1)*(2*self.nMarkers)].reshape(-1,2) 
                                R,t,lamb = rotation[maxIdx],translation[maxIdx].reshape(-1,3),scale[maxIdx]
                                P1,P2 = np.hstack((cameraMat[minIdx], [[0.], [0.], [0.]])),np.matmul(cameraMat[maxIdx], np.hstack((R, t.T)))
                                projPt1,projPt2 = projectionPoints(np.array(pts1)),projectionPoints(np.array(pts2))
                                
                                # Triangulate
                                points4d = triangulatePoints(P1.astype(float),P2.astype(float),projPt1.astype(float),projPt2.astype(float))
                                points3d = (points4d[:3, :]/points4d[3, :]).T
                                if points3d[0, 2] < 0: points3d = -points3d  
                                
                                # Project in scale regarding the minimum index camera
                                points3d = np.hstack((points3d*lamb/100,np.ones((points3d.shape[0],1)))).T
                                points3d = np.matmul(projMat[minIdx],points3d).T

                                # Translate and Rotate to ground plane
                                points3d += [0,d/b,0,0]
                                points3d = np.matmul(P_plane,points3d.T).T
                                points3d += [0,0,-h,0]
                                
                                # Save to array
                                dfTriang[k,0:(4*self.nMarkers)] = np.copy(points3d.ravel()) 
                                if not len(allPoints3d):
                                    allPoints3d = np.copy(points3d)
                                else: 
                                    allPoints3d = np.vstack((allPoints3d,points3d)) 
                                            
        finally:        
            # Close everything
            self.server_socket.close()
            destroyAllWindows()
            
            # Save results
            if self.save: 
                np.savetxt('data/camTest.csv', np.array(dfSave), delimiter=',')
            
            # Just comprising dataset if wanted to plot
            emptyLines = np.unique([i for i in range(0,dfInterp.shape[0]) if not dfInterp[i][-1]])
            dfInterp = np.delete(dfInterp,emptyLines,axis=0)
            dfTriang = np.delete(dfTriang,emptyLines,axis=0)
            print('[INFO] found ' +str(dfTriang.shape[0])+ ' interpolated pics')
            allPoints3d = np.array(allPoints3d).T

            # Prepare data for plotting
            groundData = {'planeDisplacement': d/b, 
                        'planeRotation': P_plane}

            cameraData = {'cameraHeight': h,
                        'projectionMatrices': projMat}

            # Plotting
            plotArena(title='Capture Plot Analysis', 
                      allPoints3d=allPoints3d, 
                      cameraData=cameraData, 
                      groundData=groundData)
            
# Parser for command line
parser = argparse.ArgumentParser(description='''Server for the MoCap system at the Erobotica lab of UFCG.
                                                \nPlease use it together with the corresponding client script.''',add_help=False)
parser.add_argument('-cam',type=int,help='number of active cameras in capture (default: 4)',default=4)
parser.add_argument('-marker',type=int,help='number of expected markers (default: 4)',default=4)
parser.add_argument('-trig',type=int,help='trigger time in seconds (default: 10)',default=10)
parser.add_argument('-rec',type=int,help='recording time in seconds (default: 30)',default=30)
parser.add_argument('-fps',type=int,help='interpolation fps (default: 100FPS)',default=100)
parser.add_argument('--verbose',help='show ordering and interpolation verbosity (default: off)',default=False, action='store_true')
parser.add_argument('--help', action='help', default=argparse.SUPPRESS, help='show this help message and exit.')
parser.add_argument('-save',help='save received packages to CSV (default: off)',default=False, action='store_true')
args = parser.parse_args()

mcrServer_ = mcrServer(args.cam, args.marker, args.trig, args.rec, args.fps, args.verbose, args.save)

mcrServer_.connect()
mcrServer_.collect()