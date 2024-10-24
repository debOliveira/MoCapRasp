import time, os
import picamera

# set outout folder for captured calibration image set
output_dir = os.path.expanduser('~/pics')
os.makedirs(output_dir, exist_ok=True)
os.system(f'rm -rf {output_dir}/*')

print("[INFO] Capturing calibration image set")
with picamera.PiCamera(resolution=(960,640), framerate=20,
                       sensor_mode=2) as camera:

    camera.start_preview(fullscreen=False,window=(100,100,1280,960))

    print("[INFO] Starting camera warm-up")
    time.sleep(5)
    print("[INFO] camera warm-up complete")
    camera.shutter_speed = camera.exposure_speed
    camera.exposure_mode = 'off'
    camera.color_effects = (128,128)
    g = camera.awb_gains
    camera.awb_mode = 'off'
    camera.awb_gains = g

    print("[INFO] Recording started. Capturing still images")
    max_images = 30
    count = 0
    for filename in camera.capture_continuous(f'{output_dir}/{{counter:01d}}000000000.png'):
        print('Captured %s' % filename)
        time.sleep(.5)
        count += 1
        if count >= max_images:
            break
    camera.stop_preview()
