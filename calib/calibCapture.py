import time, os
import picamera

os.system('rm -rf pics/*')    
            
print("[INFO] setting up camera")
with picamera.PiCamera(resolution=(640,480), framerate=20,
                       sensor_mode=2) as camera:
    
    camera.start_preview(fullscreen=False,window=(100,100,1280,960))

    # Give the camera some warm-up time
    time.sleep(5)
    camera.shutter_speed = camera.exposure_speed
    camera.exposure_mode = 'off'
    camera.color_effects = (128,128)
    g = camera.awb_gains
    camera.awb_mode = 'off'
    camera.awb_gains = g   
        
    print('[RECORDING]')
    for filename in camera.capture_continuous('pics/img{counter:03d}.jpg'):
        print('Captured %s' % filename)
        time.sleep(.5) # Wait 5 minutes
    camera.stop_preview()
  
