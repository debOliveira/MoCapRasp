# import time module, Observer, FileSystemEventHandler
import time
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
import cv2,os,socket
import numpy as np

os.system('rm -rf /dev/shm/*.bmp')

params = cv2.SimpleBlobDetector_Params()
params.minThreshold = 0    
params.thresholdStep = 50
params.maxThreshold = 200
params.minDistBetweenBlobs = 0
params.filterByColor = True
params.blobColor = 0
params.minRepeatability =  1
detector = cv2.SimpleBlobDetector_create(params)

pid = os.getpid()
os.system('sudo renice -n -19 -p '+str(pid))
times=[]
frames=[]

# Socket parameters
UDPSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

def imageProcessing():
    counter=0
    while True:
        start=time.time()
        img = (yield)
        counter+=1
        _,thresh = cv2.threshold(img,150,255,cv2.THRESH_BINARY)
        coord = cv2.findNonZero(thresh)
        if coord is not None:
            xMin,xMax=(coord[-1][0][1], coord[0][0][1]) if coord[0][0][1]>coord[-1][0][1] else (coord[0][0][1], coord[-1][0][1])
            yMin,yMax=(coord[-1][0][0], coord[0][0][0]) if coord[0][0][0]>coord[-1][0][0] else (coord[0][0][0], coord[-1][0][0])
            img = cv2.bitwise_not(img[xMin-5:xMax+5,yMin-5:yMax+5])
            keypoints = detector.detect(img)
            N = np.array(keypoints).shape[0]
            if N < 3:
                print('[WARNING] found only '+str(N)+' blobs')
                continue
            elif N > 3:
                diameter = np.zeros(N)
                for i in range(0,N): diameter[i] = (keypoints[i].size)
                orderAscDiameters = np.argsort(diameter)
                keypoints = [keypoints[orderAscDiameters[0]],keypoints[orderAscDiameters[1]],keypoints[orderAscDiameters[2]]]
            msg = np.zeros(10)
            for i in range(3): msg[i<<1],msg[(i<<1)+1]=keypoints[i].pt[0],keypoints[i].pt[1]
            msg[6],msg[7],msg[8],msg[9]= xMin,yMin,start,counter
            UDPSocket.sendto(msg.tobytes(),("192.168.0.103", 8888))
        times.append(time.time()-start)
          
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
            #print("Watchdog received created event - % s." % event.src_path[10:-4])
            if Handler.counter:
                img = cv2.imread('/dev/shm/'+str(Handler.counter-1).zfill(4)+'.bmp',cv2.IMREAD_GRAYSCALE)
                if img is not None: Handler.coRout.send(img)
                os.remove('/dev/shm/'+str(Handler.counter-1).zfill(4)+'.bmp')
            Handler.counter+=1
                
if __name__ == '__main__':
    watch = OnMyWatch()
    watch.run()
    times = np.array(times)
    print('[RESULTS] processing with '+str(round(1/np.mean(times),2))+'FPS')
    print('[RESULTS] '+str(len(times))+' valid images')
    '''print("Display frames with OpenCV...")
    for frame in frames:
        cv2.imshow("Slow Motion", frame)
        cv2.waitKey(10) # request maximum refresh rate

    cv2.destroyAllWindows()'''
