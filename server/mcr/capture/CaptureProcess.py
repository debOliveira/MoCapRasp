# IMPORTS >>> DO NOT CHANGE <<<
import warnings
warnings.filterwarnings('ignore')
import socket, time
import numpy as np

from mcr.misc.constants import cameraMat, distCoef

class CaptureProcess(object):
    def __init__(self,cameraids,markers,trigger,record,fps,verbose,save):
        # VARIABLES >>> DO NOT CHANGE <<<
        self.cameraids = str(cameraids).split(',')
        self.cameras = len(self.cameraids)
        self.markers = markers
        self.triggerTime = trigger
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

        self.cameraMat = np.copy(cameraMat)
        self.distCoef = np.copy(distCoef)

        # Do not change below this line, socket variables
        self.nImages = int(self.record / self.step)
        self.imageSize = []
        
        for _ in range(self.cameras): 
            self.imageSize.append([])
        
        print('[INFO] Creating server')

        self.bufferSize = 1024
        self.server_socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM) 
        self.server_socket.bind(('0.0.0.0',8888))

    # Connect with clients
    def connect(self):
        print('[INFO] Server running, waiting for clients')

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
            print('[INFO] Camera '+str(idx)+' connected at '+str(address[0]))

            # Redo intrinsics
            ret,newCamMatrix=self.intrinsics(self.cameraMat[idx],self.imageSize[idx][0],self.imageSize[idx][1],self.imageSize[idx][2])
            if ret: 
                self.cameraMat[idx]=np.copy(newCamMatrix)
            else: 
                exit()

            addedCams.append(idx)
            ports.append(address)
        
        print('[INFO] All clients connected')

        # Send trigger
        self.triggerTime += time.time()
        for i in range(self.cameras): 
            self.server_socket.sendto((str(self.triggerTime)+' '+str(self.record)).encode(),tuple(ports[i]))
        print('[INFO] Trigger sent')

    # New intrinsics
    def intrinsics(self,origMatrix,w,h,mode):
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
                print('[ERROR] Unknow conversion for intrinsics matrix')
                return False,camIntris
            return True,camIntris
        else:
            print('[ERROR] Out of proportion of the camera mode')
            return False,camIntris

    # This function is overriden at each custom capture process 
    def collect(self):
        pass