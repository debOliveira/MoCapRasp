# Fast reading from the raspberry camera with Python, Numpy, and OpenCV
# Made by @CarlosGS in May 2017
# Adapted by @debOliveira in May 2022
# License: Public Domain, attribution appreciated

import numpy as np
import subprocess as sp
import RPi.GPIO as GPIO
import time,cv2,atexit,socket,argparse,selectors

# parser for command line
parser = argparse.ArgumentParser(description='''Capture client for the MoCap system at the Erobotica lab of UFCG.
                                                \nPlease use it together with the corresponding server script.''',add_help=False)
parser.add_argument('-w',type=int,help='image width (default: 960px)',default=960)
parser.add_argument('-h',type=int,help='image height (default: 720px)',default=720)
parser.add_argument('-fps',type=int,help='frames per second (default: 40FPS)',default=40)
parser.add_argument('-ag',type=int,help='camera analog gain (default: 2)',default=2)
parser.add_argument('-dg',type=int,help='camera digital gain (default: 4)',default=4)
parser.add_argument('-md',type=int,default=4,help='camera mode (default: 4)')
parser.add_argument('--help', action='help', default=argparse.SUPPRESS, help='Show this help message and exit.')
args = parser.parse_args()
frames = [] 
N_frames = 0

# video capture parameters
(w,h) = (args.w,args.h)
bytesPerFrame = w * h
md,ag,dg,fps = args.md,args.ag,args.dg,args.fps
winH = int(h*960/w)
# video capture command
videoCmd = "./raspividyuv --save-pts - -t 0 --output - -w "+str(w)+" -h "+str(h)+" -p 0,0,960,"+str(winH)+" -md "+str(md)+" -fps "+str(fps)+" --luma -fli off -cfx 128,128 -ex off -awb off --awbgains 1.3,1.8 -ag "+str(ag)+" -dg "+str(dg)+" -co 100"
videoCmd = videoCmd.split() # Popen requires that each parameter is a separate string
print('[INFO] Size '+str(w)+'x'+str(h)+', FPS '+str(fps)+', mode '+str(md))

# turning LED on
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
led = 4
GPIO.setup(led,GPIO.OUT)
print("[INFO] LED on and parameters configured")
GPIO.output(led,1)

# server parameters
print('[INFO] connecting to server')
hostnamePC = socket.gethostbyname('debora-pc.local')
UDPSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
UDPSocket.sendto(str(str(w)+','+str(h)+','+str(md)).encode(),(hostnamePC, 8888))
message,_ = UDPSocket.recvfrom(1024)
start = float(message.split()[0])
max_frames = int(message.split()[1])*fps
print('[INFO] waiting trigger')
now = time.time()
while now < start: now = time.time()
print('[INFO] delay in sec: ',now-start)
#max_frames=20

# running command
cameraProcess = sp.Popen(videoCmd, stdout=sp.PIPE) # start the camera
atexit.register(cameraProcess.terminate) # this closes the camera process in case the python scripts exits unexpectedly
print("[INFO] RECORDING ...")

start = time.time()
while True:
	# capture frame
	frame = np.frombuffer(cameraProcess.stdout.read(bytesPerFrame), dtype=np.uint8)
	if frame.size != bytesPerFrame:
		print("[ERROR] Camera stream closed unexpectedly")
		break
	frame.shape = (h,w) 
	# capture timestamp
	ts = cameraProcess.stdout.readline()[-11:-1].decode().strip()
	# write image
	cv2.imwrite('/dev/shm/'+ts.zfill(10)+'.bmp',frame)
	# free memory
	cameraProcess.stdout.flush()
	del frame
	# count frame
	N_frames += 1
	if N_frames == max_frames: break
	
# closing buffer
end = time.time()-start
cameraProcess.terminate() 
GPIO.output(led,0)
print('[INFO] buffer closed and LED off')

# verbose
elapsed_seconds = float(ts)/1e6
print("[RESULTS] "+str(round(N_frames/elapsed_seconds,2))+" FPS (PTS) and "+
				   str(round(N_frames/end,2))+" FPS (time lib)")

### DEBUG ###
'''plt.figure(figsize=(4,3))
plt.plot(np.diff(np.array(frames).astype(float)/1e6),label='Achieved FPS period')
plt.axhline(y=1/fps,color='r',linestyle='-',label='Reference FPS period')
plt.xlim(0,np.array(frames).shape[0])
plt.legend(loc='best')
plt.grid()
plt.xlabel('Image number')
plt.ylabel('Period to previous picture (ms)')
plt.title('FPS period achieved at '+str(h)+'p, '+str(fps)+'FPS')
plt.savefig(str(h)+'p'+str(fps)+'FPS.png',dpi=300,bbox_inches="tight")

print("Writing frames to disk...")
out = cv2.VideoWriter("slow_motion.avi", cv2.cv.CV_FOURCC(*"MJPG"), 30, (w,h))
for n in range(N_frames):
    #cv2.imwrite("frame"+str(n)+".png", frames[n]) # save frame as a PNG image
    frame_rgb = cv2.cvtColor(frames[n],cv2.COLOR_GRAY2RGB) # video codec requires RGB image
    out.write(frame_rgb)
out.release()

print("Display frames with OpenCV...")
for frame in frames:
    cv2.imshow("Slow Motion", frame)
    cv2.waitKey(10) # request maximum refresh rate
    
cv2.destroyAllWindows()'''


