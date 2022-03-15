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
params.maxThreshold = 105
params.minDistBetweenBlobs = 2
params.filterByColor = True
params.filterByConvexity = False
params.minConvexity = 0.95
params.filterByCircularity = False
params.minCircularity=0.80
params.filterByArea= True
params.minArea = 10
params.filterByInertia = False
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
        _,thresh = cv2.threshold(img,220,255,cv2.THRESH_BINARY)
        coord = cv2.findNonZero(thresh)
        if coord is not None:
            coord = coord.reshape(-1,2).T
            xMin,xMax=min(coord[1]),max(coord[1])
            yMin,yMax=min(coord[0]),max(coord[0])
            img = img[xMin-10:xMax+10,yMin-10:yMax+10]
            if img.shape[0] and img.shape[1]:
                img = cv2.bitwise_not(cv2.blur(img,(3,3)))
                keypoints = detector.detect(img) 
                N = np.array(keypoints).shape[0]
                '''img=cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
                bitsShift = 4
                constMultiplier = 16
                for keyPt in keypoints:
                    center = (int(np.round(keyPt.pt[0]*constMultiplier)), int(np.round(keyPt.pt[1]*constMultiplier)))
                    radius = int(np.round(keyPt.size/2*constMultiplier))
                    imgWithKPts = cv2.circle(img, center, radius, (255,0,0), thickness = 1, lineType = 16, shift = bitsShift)
                    cv2.circle(img, center, 1, (0, 0, 255), -1,  shift = bitsShift)
                frames.append(img)'''
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
            else: print('[WARNING] error cropping image')
        else: print('[WARNING] no bright spot in image')
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
                time.sleep(20)
                self.observer.stop()
                UDPSocket.sendto(np.array([0.0]).tobytes(),("192.168.0.103", 8888))
                print("Observer Stopped 1")
                break
        except:
            UDPSocket.sendto(np.array([0.0]).tobytes(),("192.168.0.103", 8888))
            self.observer.stop()
            print("Observer Stopped 2")
  
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
    print("Display frames with OpenCV...")
    for frame in frames:
        cv2.imshow("Slow Motion", frame)
        cv2.waitKey(10) # request maximum refresh rate

    cv2.destroyAllWindows()
