import RPi.GPIO as GPIO
from picamerax import PiCamera
import cv2
import time
    
print("[INFO] init `picamera` module...")
camera = PiCamera()
# config camera and take test pic
camera.sensor_mode=7
camera.resolution=(640,480)
camera.framerate=90
camera.iso = 0
camera.start_preview(fullscreen=False)
time.sleep(10)
camera.shutter_speed = camera.exposure_speed
camera.exposure_mode='off'
#g = camera.awb_gains
camera.awb_mode = 'off'
camera.analog_gain=1
camera.awb_gains = 0.9

'''print('Analog gain:',float(camera.analog_gain))
print('Digital gain:',float(camera.analog_gain))
print('AWB gain:',camera.awb_gains)
print('Shutter speed (micros):',camera.shutter_speed)
print('Sensor mode:',camera.sensor_mode)
print('Framerate delta:',camera.framerate_delta)
print('ISO:',camera.iso)'''


print("[INFO] setting pins...")
OUT_PIN = 18
IN_PIN = 27
GPIO.setmode(GPIO.BCM)
GPIO.setup(IN_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(OUT_PIN, GPIO.OUT)

print("[INFO] sending trigger...")
for i in range(0,3):
    start = time.time_ns()
    GPIO.output(OUT_PIN, GPIO.HIGH)
    camera.capture('foo.jpg')
    print("\n[INFO] image captured - ",(time.time_ns()-start)/(10**9))
    GPIO.wait_for_edge(IN_PIN,GPIO.RISING)
    print("[INFO] ack received   - ",(time.time_ns()-start)/(10**9))
    time.sleep(0.01)
    print("[INFO] down cmd       - ",(time.time_ns()-start)/(10**9))
    GPIO.output(OUT_PIN, GPIO.LOW)
    print("[INFO] pinout down    - ",(time.time_ns()-start)/(10**9))
    time.sleep(0.01)
    print("[INFO] loop finished  - ",(time.time_ns()-start)/(10**9))
    
print("[INFO] turning off...")
GPIO.cleanup()
camera.stop_preview()

frame = cv2.imread('foo.jpg')
frame = cv2.cvtColor(frame,cv2.COLOR_RGB2GRAY)
cv2.imshow('Frame',frame)
cv2.waitKey(0)
cv2.destroyAllWindows()
