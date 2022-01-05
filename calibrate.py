from time import sleep
from picamera import PiCamera
import os

os.system('rm -rf calibPics/*')

camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 30
camera.sensor_mode = 7
camera.start_preview()
# allow the camera to warmup
print('sleeping...')
time.sleep(5)
print('awake')
camera.exposure_mode = 'off'
g = camera.awb_gains
camera.awb_mode = 'off'
camera.awb_gains = g
i = 0

for filename in camera.capture_continuous('calibPics/img{counter:03d}.jpg'):
    print('Captured %s' % filename)
    sleep(2) # wait 5 minutes
    if i == 10:
        break
    i+=1
    
os.system('zip -r calib.zip calibPics/')