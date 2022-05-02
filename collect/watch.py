# import time module, Observer, FileSystemEventHandler
import time
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
import cv2,os,socket
import numpy as np

os.system('rm -rf /dev/shm/*.bmp')

params = cv2.SimpleBlobDetector_Params()
params.minThreshold = 50   
params.thresholdStep = 50
params.maxThreshold = 151
params.minDistBetweenBlobs = 0
params.filterByColor = True
params.filterByArea= True
params.minArea = 2
params.filterByConvexity = False
params.minConvexity = 0.90
params.blobColor = 0
params.minRepeatability = 3
detector = cv2.SimpleBlobDetector_create(params)

pid = os.getpid()
os.system('sudo renice -n -19 -p '+str(pid))
times=[]
frames=[]

# Socket parameters
UDPSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

def imageProcessing():
    counter=0          
    bitsShift = 4
    constMultiplier = 16      
    while True:
        try:
            start=time.time()
            img,ts = (yield)
            _,thresh = cv2.threshold(img,127,255,cv2.THRESH_BINARY)
            coord = cv2.findNonZero(thresh).reshape(-1,2).T
            xMin,xMax=min(coord[1]),max(coord[1])
            yMin,yMax=min(coord[0]),max(coord[0]) 
            keypoints = detector.detect(cv2.bitwise_not(img[xMin-10:xMax+10,yMin-10:yMax+10]))
            N = np.array(keypoints).shape[0]
            msg = np.zeros(N*3+4)
            for i in range(N): 
                msg[(i<<1)+i],msg[(i<<1)+i+1],msg[(i<<1)+i+2]=keypoints[i].pt[0],keypoints[i].pt[1],keypoints[i].size
            msg[-4],msg[-3],msg[-2],msg[-1]= xMin,yMin,ts,counter
            '''imgWithKPts = cv2.cvtColor(img[xMin-10:xMax+10,yMin-10:yMax+10], cv2.COLOR_GRAY2BGR)
            for keyPt in keypoints:
                center = (int(np.round(keyPt.pt[0]*constMultiplier)), int(np.round(keyPt.pt[1]*constMultiplier)))
                radius = int(np.round(keyPt.size/2*constMultiplier))
                imgWithKPts = cv2.circle(imgWithKPts, center, radius, (255,0,0), thickness = 1, lineType = 16, shift = bitsShift)
                cv2.circle(imgWithKPts, center, 1, (0, 0, 255), -1,  shift = bitsShift)     
            frames.append(imgWithKPts)'''    
            times.append(time.time()-start)
            counter+=1
        except GeneratorExit: return
        except: continue
          
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
                time.sleep(120)
                self.observer.stop()
                UDPSocket.sendto(np.array([0.0]).tobytes(),("192.168.0.104", 8888))
                print("Observer Stopped 1")
                break
        except:
            UDPSocket.sendto(np.array([0.0]).tobytes(),("192.168.0.104", 8888))
            self.observer.stop()
            print("Observer Stopped 2")
  
        self.observer.join()
  
  
class Handler(FileSystemEventHandler):
    counter = 0  
    coRout = imageProcessing()
    coRout.__next__()
    lastImg = ''
    
    @staticmethod
    def on_any_event(event):
        if event.is_directory:
            return None       
        elif event.event_type == 'created':
            #print("Watchdog received created event - % s." % event.src_path)
            if Handler.counter:
                name = Handler.lastImg
                img = cv2.imread('/dev/shm/'+name+'.bmp',cv2.IMREAD_GRAYSCALE)
                if img is not None: Handler.coRout.send((img,int(name)))
                os.remove('/dev/shm/'+name+'.bmp')
            Handler.lastImg = event.src_path[-13:-4]
            Handler.counter+=1
                    
                    
if __name__ == '__main__':
    watch = OnMyWatch()
    watch.run()
    if (len(times)):
        times = np.array(times[1:])
        print('[RESULTS] processing with '+str(round(1/np.mean(times),2))+'FPS')
        print('[RESULTS] '+str(len(times))+' valid images')
    else: print('[RESULTS] no valid images captured')
    print("Display frames with OpenCV...")
    for frame in frames:
        cv2.imshow("Slow Motion", frame)
        cv2.waitKey(10) # request maximum refresh rate

    cv2.destroyAllWindows()
