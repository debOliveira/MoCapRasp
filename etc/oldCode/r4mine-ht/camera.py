import RPi.GPIO as GPIO
from picamerax import PiCamera
import cv2
import time
    
print("[INFO] init `picamera` module...")
camera = PiCamera()
# config camera and take test pic
camera.sensor_mode=7
camera.resolution=(640,480)
camera.framerate=60
camera.iso = 0
camera.start_preview(fullscreen=False)
time.sleep(10)
camera.shutter_speed = camera.exposure_speed
camera.exposure_mode='off'
#g = camera.awb_gains
camera.awb_mode = 'off'
camera.analog_gain=1
camera.awb_gains = 0.9

print("[INFO] setting pins...")
OUT_PIN = 18
GPIO.setmode(GPIO.BCM)
GPIO.setup(OUT_PIN, GPIO.OUT)

print("[INFO] sending trigger...")
GPIO.output(OUT_PIN, GPIO.HIGH)
camera.capture('foo.jpg')
time.sleep(0.1)
GPIO.output(OUT_PIN, GPIO.LOW)
time.sleep(0.1)
    
print("[INFO] turning off...")
GPIO.cleanup()
camera.stop_preview()

print('Analog gain:',float(camera.analog_gain))
print('Digital gain:',float(camera.analog_gain))
print('AWB gain:',camera.awb_gains)
print('Shutter speed (micros):',camera.shutter_speed)
print('Sensor mode:',camera.sensor_mode)
print('Framerate delta:',camera.framerate_delta)
print('ISO:',camera.iso)

frame = cv2.imread('foo.jpg')
frame = cv2.cvtColor(frame,cv2.COLOR_RGB2GRAY)
cv2.imshow('Frame',frame)
cv2.waitKey(0)
cv2.destroyAllWindows()