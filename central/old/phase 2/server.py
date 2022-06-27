import io
import socket
from socketserver import ThreadingUDPServer
import struct
from PIL import Image
import cv2
import numpy as np
import threading


class myTCPServer(object):
    def __init__(self,number):
        print('[INFO] starting server '+str(number))
        self.count=0
        self.server_socket = socket.socket()
        self.server_socket.bind(('0.0.0.0', 7999+number))
        self.server_socket.listen(0)
        self.number=number
        # BLOB DETECTOR
        self.params = cv2.SimpleBlobDetector_Params()
        self.params.minThreshold = 0    
        self.params.thresholdStep = 100
        self.params.maxThreshold = 200
        self.params.filterByArea = True
        self.params.minArea = 2
        self.params.filterByConvexity = False
        self.params.minConvexity = 0.95
        self.params.minDistBetweenBlobs = 0
        self.params.filterByColor = True
        self.params.blobColor = 0
        self.params.minRepeatability =  1
        self.detector = cv2.SimpleBlobDetector_create(self.params)
        self.capturing = True
        self.myUDPServer_ = myUDPServer(number)
    
    def run(self):
        t = threading.Thread(target=self.myUDPServer_.run,args=[self])
        t.start()
        # Accept a single connection and make a file-like object out of it
        connection = self.server_socket.accept()[0].makefile('rb')
        print('[INFO] client connected')
        try:
            while True:
                # Read the length of the image as a 32-bit unsigned int. If the
                # length is zero, quit the loop
                image_len = struct.unpack('<L', connection.read(struct.calcsize('<L')))[0]
                if not image_len:
                    break
                # Construct a stream to hold the image data and read the image
                # data from the connection
                image_stream = io.BytesIO()
                image_stream.write(connection.read(image_len))
                # Rewind the stream, open it as an image with PIL and do some
                # processing on it
                image_stream.seek(0)
                img = np.array(Image.open(image_stream).convert("L"))
                nonNullCoord = img > 200
                if np.any(nonNullCoord):
                        a,b = np.nonzero(nonNullCoord.argmax(1))[0], np.nonzero(nonNullCoord.argmax(0))[0]
                        imgMasked = cv2.bitwise_not(img[a[0]-5:a[-1]+5,b[0]-5:b[-1]+5])
                        keypoints = self.detector.detect(imgMasked)
                        N = np.array(keypoints).shape[0]
                        if N < 3:
                            print('[WARNING] found only '+str(N)+' blobs')
                            continue
                        elif N > 3:
                            print('[WARNING] found '+str(N)+' blobs')
                            diameter = np.zeros(N)
                            for i in range(0,N): diameter[i] = (keypoints[i].size)
                            orderAscDiameters = np.argsort(diameter)
                            keypoints = [keypoints[orderAscDiameters[0]],keypoints[orderAscDiameters[1]],keypoints[orderAscDiameters[2]]]
                        #for i in range(3):
                            #print(keypoints[i].pt[0],keypoints[i].pt[1])
                self.count+=1
        finally:
            connection.close()
            self.server_socket.close()
            print('[FINISHED] server '+str(self.number)+' >> '+str(self.count)+' images captured')
            self.capturing=False
            t.join()
            print('[INFO] closed UDP server '+str(self.number)+' thread')

class myUDPServer(object):
    def __init__(self,number):
        self.server_socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        self.server_socket.bind(('0.0.0.0',8799+number))
        print('[INFO] starting UDP server '+str(number))
    
    def run(self,owner):
        try:
            #while owner.capturing:
            pass
                #timestamp = np.frombuffer(self.server_socket.recvfrom(1024),dtype=np.float64)
                #print(owner.number, timestamp)
        finally:
            self.server_socket.close()            
class myServers(object):
    def __init__(self):
        self.numberCameras,self.myThreads = 2,[]
        for i in range(self.numberCameras):
            server = myTCPServer(number=i+1)
            t = threading.Thread(target=server.run, args=[])
            t.start()
            self.myThreads.append(t)
    def __exit__(self):
        for i in range(self.numberCameras):
            t = self.myThreads[i]
            t.join()
            print('[INFO] closed TCP server '+str(i)+' thread')

myServers_ = myServers()