# Fast reading from the raspberry camera with Python, Numpy, and OpenCV
# Allows to process grayscale video up to 124 FPS (tested in Raspberry Zero Wifi with V2.1 camera)
#
# Made by @CarlosGS in May 2017
# Club de Robotica - Universidad Autonoma de Madrid
# http://crm.ii.uam.es/
# License: Public Domain, attribution appreciated

import numpy as np
import subprocess as sp
import time,cv2,atexit,socket

print('[INFO] configuring parameters')
frames = [] # stores the video sequence for the demo
N_frames = 0
# Video capture parameters
(w,h) = (960,720)
bytesPerFrame = w * h
md = 4
fps = 1 # setting to 250 will request the maximum framerate possible
ag,dg = 8,1

videoCmd = "./raspividyuv -p 0,0,960,720 -w "+str(w)+" -h "+str(h)+" -cfx 128,128 --output - --timeout 0 --framerate "+str(fps)+" -fli off -ex off --luma -awb off --awbgains 1.3,1.8 -ag "+str(ag)+" -dg "+str(dg)+" -pts - -md "+str(md)
videoCmd = videoCmd.split() # Popen requires that each parameter is a separate string
max_frames=200

cameraProcess = sp.Popen(videoCmd, stdout=sp.PIPE) # start the camera
atexit.register(cameraProcess.terminate) # this closes the camera process in case the python scripts exits unexpectedly
print("RECORDING ...")

while True:
    cameraProcess.stdout.flush() # discard any frames that we were not able to process in time
    # Parse the raw stream into a numpy array
    frame = np.frombuffer(cameraProcess.stdout.read(bytesPerFrame), dtype=np.uint8)
    if frame.size != bytesPerFrame:
        print("Error: Camera stream closed unexpectedly")
        break
    frame.shape = (h,w) # set the correct dimensions for the numpy array
    ts = np.array(cameraProcess.stdout.read(9).decode("utf-8"), dtype="U9")
    cv2.imwrite('/dev/shm/'+str(N_frames).zfill(4)+'.bmp',frame)
    del frame # free the allocated memory
    N_frames += 1
    if N_frames == max_frames: break
    
end_time = time.time()
cameraProcess.terminate() # stop the camera
elapsed_seconds = float(ts)/1e6
print("[RESULTS] "+str(N_frames/elapsed_seconds)+" FPS")

'''print("Display frames with OpenCV...")
for frame in frames:
    cv2.imshow("Slow Motion", frame)
    cv2.waitKey(10) # request maximum refresh rate
    
cv2.destroyAllWindows()'''
