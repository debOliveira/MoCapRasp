import io
import time
import picamera
import os
os.system('rm -rf pics/*')

recTime = 2

class SplitFrames(object):
    def __init__(self):
        self.frame_num = 0
        self.output = None

    def write(self, buf):
        if buf.startswith(b'\xff\xd8'):
            # Start of new frame; close the old one (if any) and
            # open a new output
            if self.output:
                self.output.close()
            self.frame_num += 1
            self.output = io.open('bg.jpg', 'wb')
        self.output.write(buf)

with picamera.PiCamera(resolution=(640,480), framerate=40,
                       sensor_mode=4) as camera:
    camera.start_preview(fullscreen=False,window=(100,100,640,480))
    print("[INFO] setting up camera")
    # Give the camera some warm-up time
    time.sleep(5)
    camera.shutter_speed = camera.exposure_speed
    camera.exposure_mode = 'off'
    camera.color_effects = (128,128)
    g = camera.awb_gains
    camera.awb_mode = 'off'
    camera.awb_gains = g
    output = SplitFrames()
    
    print('[RECORDING..]')
    start = time.time()
    camera.start_recording(output, format='mjpeg')
    camera.wait_recording(recTime)
    camera.stop_recording()
    finish = time.time()
print('Captured %d frames at %.2ffps' % (
    output.frame_num,
    output.frame_num / (finish - start)))