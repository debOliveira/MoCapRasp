import numpy as np
import subprocess as sp
import RPi.GPIO as GPIO
import time,cv2,atexit,socket,argparse

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

# video capture parameters 422762'
(w,h) = (args.w,args.h)
bytesPerFrame = w * h
md,ag,dg,fps = args.md,args.ag,args.dg,args.fps
winH = int(h*960/w)
# video capture command
videoCmd = "./raspividyuv --save-pts - -t 0 --output - -w "+str(w)+" -h "+str(h)+" -md "+str(6)+" -fps "+str(fps)+" --nopreview --luma"# -fli off -cfx 128,128 -ex off --luma -awb off --awbgains 1.3,1.8 -ag "+str(ag)+" -dg "+str(dg)+" -co 100"
videoCmd = videoCmd.split() # Popen requires that each parameter is a separate string
max_frames = 1

cameraProcess = sp.Popen(videoCmd, stdout=sp.PIPE, stderr=sp.PIPE) # start the camera
atexit.register(cameraProcess.terminate) # this closes the camera process in case the python scripts exits unexpectedly


frame = np.frombuffer(cameraProcess.stdout.read(bytesPerFrame), dtype=np.uint8)
value = cameraProcess.stdout.read(10)
if list(value) == list(b'         0'):
	print('>>>>>>>>>>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<<<<<<<<<<<<')
	print('>>>>>> VALID RESOLUTION-MODE-FPS COMBINATION <<<<<<')
	print('>>>>>>>>>>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<<<<<<<<<<<<')
else:
	print('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
	print('!!!!!! NON VALID RESOLUTION-MODE-FPS COMBINATION !!!!!!')
	print('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')

