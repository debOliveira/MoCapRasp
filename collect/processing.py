import io,socket
import time
import threading
import picamera
import cv2
from PIL import Image
import numpy as np

class ImageProcessor(threading.Thread):
    def __init__(self, owner):
        super(ImageProcessor, self).__init__()
        self.stream = io.BytesIO()
        self.event = threading.Event()
        self.terminated = False
        self.owner = owner
        self.start()

    def run(self):
        print('[INFO] starting thread')
        # This method runs in a separate thread
        while not self.terminated:
            # Wait for an image to be written to the stream
            if self.event.wait(1):
                try:
                    self.stream.seek(0)
                    # Read the image and do some processing on it
                    img = cv2.cvtColor(np.array(Image.open(self.stream)),cv2.COLOR_RGB2GRAY)
                    nonNullCoord = img > 180
                    if np.any(nonNullCoord):
                        a,b = np.nonzero(nonNullCoord.argmax(1))[0], np.nonzero(nonNullCoord.argmax(0))[0]
                        imgMasked = cv2.bitwise_not(img[a[0]-5:a[-1]+5,b[0]-5:b[-1]+5])
                        keypoints = self.owner.detector.detect(imgMasked)
                        N = np.array(keypoints).shape[0]
                        if N < 3:
                            print('lower blob threshold')
                            continue
                        elif N > 3:
                            for i in range(0,N): diameter[i] = (keypoints[i].size)
                            orderAscDiameters = np.argsort(diameter)
                            keypoints = [keypoints[orderAscDiameters[0]],keypoints[orderAscDiameters[1]],keypoints[orderAscDiameters[2]]]
                        msg = np.zeros(9)
                        for i in range(3):
                            msg[i<<1],msg[(i<<1)+1]=keypoints[i].pt[0],keypoints[i].pt[1]
                        msg[6],msg[7],msg[8] = a[0],b[0],time.time_ns()
                    # Set done to True if you want the script to terminate
                    # at some point
                    #self.owner.done=True
                finally:
                    # Reset the stream and event
                    self.stream.seek(0)
                    self.stream.truncate()
                    self.event.clear()
                    # Return ourselves to the available pool
                    with self.owner.lock:
                        if N>=3: self.owner.UDPSocket.sendto(msg.tobytes(),("192.168.0.103", 8888))
                        self.owner.count += 1
                        self.owner.pool.append(self)

class ProcessOutput(object):
    def __init__(self):
        self.done = False
        # Construct a pool of 4 image processors along with a lock
        # to control access between threads
        self.lock = threading.Lock()
        self.pool = [ImageProcessor(self) for i in range(4)]
        self.processor = None
        # IMAGE CAPTURE
        self.count = 0
        # BLOB DETECTOR
        self.params = cv2.SimpleBlobDetector_Params()
        self.params.minThreshold = 0    
        self.params.thresholdStep = 100
        self.params.maxThreshold = 200
        self.params.filterByArea = True
        self.params.minArea = 2
        self.params.filterByConvexity = True
        self.params.minConvexity = 0.95
        self.params.minDistBetweenBlobs = 0
        self.params.filterByColor = True
        self.params.blobColor = 0
        self.params.minRepeatability =  1
        self.detector = cv2.SimpleBlobDetector_create(self.params)
        # UDP SERVER
        self.UDPSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        
    def write(self, buf):
        if buf.startswith(b'\xff\xd8'):
            # New frame; set the current processor going and grab
            # a spare one
            if self.processor:
                self.processor.event.set()
            with self.lock:
                if self.pool:
                    self.processor = self.pool.pop()
                else:
                    # No processor's available, we'll have to skip
                    # this frame; you may want to print a warning
                    # here to see whether you hit this case
                    ('[WARNING] skipping frame, no process available')
                    self.processor = None
        if self.processor:
            self.processor.stream.write(buf)

    def flush(self):
        print('[INFO] flushing, please wait ...')
        # When told to flush (this indicates end of recording), shut
        # down in an orderly fashion. First, add the current processor
        # back to the pool
        if self.processor:
            with self.lock:
                self.pool.append(self.processor)
                self.processor = None 
        # Now, empty the pool, joining each thread as we go
        while len(self.pool):
            with self.lock:
                try:
                    proc = self.pool.pop()
                except IndexError:
                    pass # pool is empty
            proc.terminated = True
            proc.join()
        self.UDPSocket.sendto(np.array([0.0]).tobytes(),("192.168.0.103", 8888))

with picamera.PiCamera(resolution=(640,480), framerate=40,
                       sensor_mode=4) as camera:
    camera.start_preview()
    time.sleep(5)
    camera.shutter_speed = camera.exposure_speed
    camera.exposure_mode = 'off'
    camera.color_effects = (128,128)
    g = camera.awb_gains
    camera.awb_mode = 'off'
    camera.awb_gains = g
    recTime = 2
    output = ProcessOutput()
    print('start recording')
    camera.start_recording(output, format='mjpeg')
    #while not output.done:
    camera.wait_recording(recTime)
    camera.stop_recording()
    print('end recording')

print('Captured %d frames at %.2ffps' % (
    output.count,
    output.count / (recTime)))
