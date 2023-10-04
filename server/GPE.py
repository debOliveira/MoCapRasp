# IMPORTS >>> DO NOT CHANGE <<<
import warnings
warnings.filterwarnings('ignore')
import socket,time,click
import numpy as np
from cv2 import destroyAllWindows,triangulatePoints

from mcr.math import findPlane
from mcr.cameras import projectionPoints
from mcr.markers import processCentroids, getOrderPerEpiline
from mcr.plot import plotArena
from mcr.constants import cameraMat, distCoef

class GPE(object):
    def __init__(self,cameraids, markers,trigger,record,fps,verbose,save):
        # VARIABLES >>> DO NOT CHANGE <<<
        self.cameraids = str(cameraids).split(',')
        self.cameras = len(self.cameraids)
        self.markers = markers
        self.trigger = trigger
        self.record = record
        self.fps = fps
        self.step = 1 / fps
        self.verbose = verbose
        self.save = save
        self.ipList = []

        # IP lookup from hostname
        try:
            self.ipList = [socket.gethostbyname(f'cam{idx}.local') for idx in self.cameraids]
        except socket.gaierror as e:
            print('[ERROR] Number of cameras do not match the number of IPs found')
            exit()

        # Do not change below this line, socket variables
        self.nImages = int(self.record / self.step)
        self.imageSize = []
        
        for _ in range(self.cameras): 
            self.imageSize.append([])
        
        print('[INFO] creating server')

        self.bufferSize = 1024
        self.server_socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM) 
        self.server_socket.bind(('0.0.0.0',8888))

    # Connect with clients
    def connect(self):
        print('[INFO] server running, waiting for clients')

        addedCams,ports=[],[]
        while len(addedCams)!=self.cameras:
            # Collect adresses
            message,address = self.server_socket.recvfrom(self.bufferSize)

            # Check if it is in IP list
            if address[0] not in self.ipList:
                print('[ERROR] IP '+address[0]+' not in the list')
                exit()

            # Get image size
            idx = self.ipList.index(address[0])
            self.imageSize[idx] = np.array(message.decode('utf-8').split(',')).astype(np.int)
            print('[INFO] camera '+str(idx)+' connected at '+str(address[0]))

            # Redo intrinsics
            ret,newCamMatrix=self.mcrIntrinsics(self.cameraMat[idx],self.imageSize[idx][0],self.imageSize[idx][1],self.imageSize[idx][2])
            if ret: 
                self.cameraMat[idx]=np.copy(newCamMatrix)
            else: 
                exit()

            addedCams.append(idx)
            ports.append(address)
        
        print('[INFO] all clients connected')

        # Send trigger
        self.trigger += time.time()
        for i in range(self.cameras): 
            self.server_socket.sendto((str(self.trigger)+' '+str(self.record)).encode(),tuple(ports[i]))
        print('[INFO] trigger sent')

    # New intrinsics
    def mcrIntrinsics(self,origMatrix,w,h,mode):
        camIntris = np.copy(origMatrix) # Copy to avoid register error
        
        # Check if image is at the available proportion
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
                print('conversion for intrinsics matrix not known')
                return False,camIntris
            return True,camIntris
        else:
            print('out of proportion of the camera mode')
            return False,camIntris

    # Collect points from clients, order and trigger interpolation
    def collect(self):
        # Internal variables
        print('[INFO] waiting capture')
        capture,counter = np.ones(self.cameras,dtype=np.bool),np.zeros(self.cameras,dtype=np.int8)
        dfSave,dfOrig = [],[]

        for _ in range(self.cameras): # For each camera
            dfOrig.append([]) # List for each camera

        # Capture loop
        try:
            while np.any(capture):
                #  Receive message
                bytesPair = self.server_socket.recvfrom(self.bufferSize)
                message = np.frombuffer(bytesPair[0],dtype=np.float64)
                address,sizeMsg = bytesPair[1],len(message)
                idx = self.ipList.index(address[0])
                if not (sizeMsg-1): capture[idx] = 0

                if capture[idx]: # Check if message is valid
                    msg = message[0:sizeMsg-4].reshape(-1,3)
                    coord = msg[:,0:2]

                    # Store message parameters
                    a,b,timeNow,imgNumber = message[-4],message[-3],message[-2],int(message[-1]) 
                    if not len(coord): continue

                    # Undistort points
                    undCoord = processCentroids(coord,a,b,self.cameraMat[idx],distCoef[idx])
                    if undCoord.shape[0]==3:
                        if self.save: dfSave.append(np.concatenate((undCoord.reshape(undCoord.shape[0]*undCoord.shape[1]),[timeNow,imgNumber,idx])))
                        if not counter[idx]: dfOrig[idx] = np.hstack((undCoord.reshape(6),timeNow))
                        counter[idx]+=1

                    # Do I have enough points?     
                    if np.all(counter > 0): break                            
        finally:        
            # Close everything
            self.server_socket.close()
            destroyAllWindows()

            # Save results
            if self.save: 
                np.savetxt('data/camGround.csv', np.array(dfSave), delimiter=',')

            # Import R,T and lambda
            rotation = np.genfromtxt('data/R.csv', delimiter=',').reshape(-1,3,3)
            translation = np.genfromtxt('data/t.csv', delimiter=',').reshape(-1,1,3)
            projMat = np.genfromtxt('data/projMat.csv', delimiter=',').reshape(-1,4,4)
            scale,FMatrix = np.genfromtxt('data/lamb.csv', delimiter=','), np.genfromtxt('data/F.csv', delimiter=',').reshape(-1,3,3)
            
            # Order centroids
            for j in range(self.cameras-1):
                # Collect extrinsics from calibration between camera 0 and 1
                F = FMatrix[j]
                R,t,lamb = rotation[j+1],translation[j+1].reshape(-1,3),scale[j+1]
                
                # Order centroids per epipolar line
                pts1,pts2 = dfOrig[j][0:6].reshape(-1,2),dfOrig[j+1][0:6].reshape(-1,2)
                orderSecondFrame = getOrderPerEpiline(pts1,pts2,3,np.copy(F))
                pts2 = np.copy(pts2[orderSecondFrame])

                # Save dataset
                dfOrig[j+1][0:6] = pts2.copy().ravel()

            # Triangulate ordered centroids from the first pair
            pts1,pts2 = np.copy(dfOrig[0][0:6].reshape(-1,2)),np.copy(dfOrig[1][0:6].reshape(-1,2))
            R,t,lamb = rotation[1],translation[1].reshape(-1,3),scale[1]
            P1,P2 = np.hstack((cameraMat[0], [[0.], [0.], [0.]])),np.matmul(cameraMat[1], np.hstack((R, t.T)))
            projPt1,projPt2 = projectionPoints(np.array(pts1)),projectionPoints(np.array(pts2))
            points4d = triangulatePoints(P1.astype(float),P2.astype(float),projPt1.astype(float),projPt2.astype(float))
            points3d = (points4d[:3, :]/points4d[3, :]).T
            if points3d[0, 2] < 0: 
                points3d = -points3d
            allPoints3d = points3d*lamb/100

            # Get ground plane coefficients
            plane = findPlane(allPoints3d[0],allPoints3d[1],allPoints3d[2])
            if np.any(plane[0:3]<0): 
                plane = findPlane(allPoints3d[0],allPoints3d[2],allPoints3d[1])
            [a,b,c,d] = plane

            # Get the orthonogal vector to the plane
            v,k = np.array([a,b,c]),np.array([0,1,0])

            # Compute the angle between the plane and the y axis
            cosPhi = np.dot(v,k)/(np.linalg.norm(v)*np.linalg.norm(k))

            # Compute the versors
            [u1,u2,u3] = np.cross(v,k)/np.linalg.norm(np.cross(v,k))

            # Get the rotation matrix and new ground plane coefficients
            sinPhi = np.sqrt(1-pow(cosPhi,2))
            R_plane = np.array([
                    [cosPhi+u1*u1*(1-cosPhi),u1*u2*(1-cosPhi)-u3*sinPhi,u2*sinPhi+u1*u3*(1-cosPhi)],
                    [u1*u2*(1-cosPhi)+u3*sinPhi,cosPhi+u2*u2*(1-cosPhi),u2*u3*(1-cosPhi)-u1*sinPhi],
                    [u1*u3*(1-cosPhi)-u2*sinPhi,u2*u3*(1-cosPhi)+u1*sinPhi,cosPhi+u3*u3*(1-cosPhi)]])
            [A,B,C] = np.matmul(R_plane,np.array([a,b,c]).T)
            
            # Preparing ground data 
            P_plane = np.vstack((np.hstack((R_plane,np.array([0,0,0]).reshape(3,-1))),np.hstack((np.zeros((3)),1))))
            newPlane = np.array([A,B,C])

            groundData = {'planeDisplacement': d/b, 
                        'planeRotation': P_plane,
                        'planeCoefficients': newPlane}

            # Preparing camera data
            o = np.matmul(projMat[0],[[0.],[0.],[0.],[1]]).ravel()
            o += [0,d/b,0,0] # Displace in relation to ground
            o = np.matmul(P_plane,o).ravel() # Rotate in relation to ground
            h = o[2] # Let 'h' be the height of the 0th camera

            cameraData = {'cameraHeight': h,
                        'projectionMatrices': projMat}

            # Preparing captured data 
            allPoints3d = np.hstack((allPoints3d,np.ones((allPoints3d.shape[0],1))))
            allPoints3d += [0,d/b,0,0]
            allPoints3d = np.matmul(P_plane,allPoints3d.T).T
            allPoints3d += [0,0,-h,0]
            allPoints3d = allPoints3d.T

            plotArena(title='Ground Plane Estimation', 
                      allPoints3d=allPoints3d, 
                      cameraData=cameraData, 
                      groundData=groundData)

            # Save data
            np.savetxt('data/P_plane.csv', np.array(P_plane), delimiter=',')
            np.savetxt('data/groundData.csv', np.array([d,b,h]), delimiter=',')

# Parser for command line
@click.command(name="gpe")
@click.option('--cameraids', '-c', default = '0,1,2,3', help = 'List of active camera IDs (Default: 0,1,2,3)')
@click.option('--markers',   '-m', default = 3,         help = 'Number of expected markers (Default: 3)')
@click.option('--trigger',   '-t', default = 2,         help = 'Trigger time in seconds (Default: 2)')
@click.option('--record',    '-r', default = 10,        help = 'Recording time in seconds (Default: 10)')
@click.option('--fps',       '-f', default = 100,       help = 'Interpolation FPS (Default: 100)')
@click.option('--verbose',   '-v', is_flag = True,      help = 'Show ordering and interpolation verbosity')
@click.option('--save',      '-s', is_flag = True,      help = 'Save received packages to CSV')
def gpe(cameraids, markers, trigger, record, fps, verbose, save):
    """
    GPE: Ground Plane Estimation
    """
    gpeServer = GPE(cameraids, markers, trigger, record, fps, verbose, save)
    gpeServer.connect()
    gpeServer.collect()