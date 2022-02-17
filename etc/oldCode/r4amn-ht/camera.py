import RPi.GPIO as GPIO
import time
from picamerax import PiCamera
import cv2
import time
    
print("[INFO] init `picamera` module...")
camera = PiCamera()
camera.sensor_mode = 7
camera.resolution = (640,480)
camera.framerate = 90
camera.iso = 0
camera.start_preview(fullscreen=False)
time.sleep(10)
camera.shutter_speed=camera.exposure_speed
camera.exposure_mode='off'
camera.awb_mode='off'
#g = camera.awb_gains
camera.digital_gain=1
camera.awb_gains=0.9

print("[INFO] configuring read pin and event trigger")
IN_PIN = 18
GPIO.setmode(GPIO.BCM)
GPIO.setup(IN_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
camera.exposure_mode='off'

print("[INFO] waiting trigger")
GPIO.wait_for_edge(IN_PIN, GPIO.RISING)
camera.capture('foo.jpg')
print("[INFO] camera triggered!")
camera.stop_preview()
GPIO.cleanup()
frame = cv2.imread('foo.jpg')
frame = cv2.cvtColor(frame,cv2.COLOR_RGB2GRAY)
cv2.imshow("Frame", frame)
cv2.waitKey(0)
cv2.destroyAllWindows()

