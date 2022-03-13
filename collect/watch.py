# import time module, Observer, FileSystemEventHandler
import time
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
import cv2,os
import numpy as np

os.system('rm -rf /dev/shm/*.bmp')
N_frames = 300
params = cv2.SimpleBlobDetector_Params()
params.minThreshold = 0    
params.thresholdStep = 100
params.maxThreshold = 200
params.filterByArea = True
params.minArea = 2
params.minDistBetweenBlobs = 0
params.filterByColor = True
params.blobColor = 0
params.minRepeatability =  1
detector = cv2.SimpleBlobDetector_create(params)
pid = os.getpid()
os.system('sudo renice -n -19 -p '+str(pid))

def imageProcessing():
    while True:
        start=time.time()
        img = (yield)
        _,img = cv2.threshold(img,242,245,cv2.THRESH_BINARY)
        coord = cv2.findNonZero(img)
        img = img[coord[0][0][1]:coord[-1][0][1],coord[0][0][0]:coord[-1][0][0]]
        keypoints = detector.detect(img)
        print(img.shape,time.time()-start)
  
class OnMyWatch:
    # Set the directory on watch
    watchDirectory = "/dev/shm/"
  
    def __init__(self):
        self.observer = Observer()
  
    def run(self):
        event_handler = Handler()
        self.observer.schedule(event_handler, self.watchDirectory, recursive = True)
        self.observer.start()
        try:
            while True:
                time.sleep(10)
                self.observer.stop()
                os.remove('/dev/shm/'+str(N_frames).zfill(4)+'.bmp')
                print("Observer Stopped")
                break
        except:
            self.observer.stop()
            print("Observer Stopped")
  
        self.observer.join()
  
  
class Handler(FileSystemEventHandler):
    counter = 0  
    coRout = imageProcessing()
    coRout.__next__()
    
    @staticmethod
    def on_any_event(event):
        if event.is_directory:
            return None       
        elif event.event_type == 'created':
            print("Watchdog received created event - % s." % event.src_path[10:-4])
            if Handler.counter:
                img = cv2.imread('/dev/shm/'+str(Handler.counter-1).zfill(4)+'.bmp',cv2.IMREAD_GRAYSCALE)
                if img is not None: Handler.coRout.send(img)
                os.remove('/dev/shm/'+str(Handler.counter-1).zfill(4)+'.bmp')
            Handler.counter+=1
                
if __name__ == '__main__':
    watch = OnMyWatch()
    watch.run()
