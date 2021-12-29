import time
import picamera

frames = 60

def filenames():
    frame = 0
    while frame < frames:
        yield 'image%02d.jpg' % frame
        frame += 1

with picamera.PiCamera(resolution=(640,480),
                       framerate=65,
                       sensor_mode=7) as camera:
    camera.start_preview()
    # Give the camera some warm-up time
    time.sleep(2)
    camera.shutter_speed = camera.exposure_speed
    camera.exposure_mode = 'sports'
    camera.awb_mode = 'off'
    camera.awb_gains = 0.9
    start = time.time()
    camera.capture_sequence(filenames(), use_video_port=True)
    finish = time.time()
print('Captured %d frames at %.2ffps' % (
    frames,
    frames / (finish - start)))
