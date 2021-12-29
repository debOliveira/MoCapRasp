import RPi.GPIO as GPIO
import time
from picamerax import PiCamera
import cv2
import time
    
print("[INFO] init `picamera` module...")
camera = PiCamera()
camera.sensor_mode = 7
camera.resolution = (640,480)
camera.framerate = 60
camera.iso = 0
camera.start_preview(fullscreen=False)
time.sleep(10)
camera.shutter_speed=camera.exposure_speed
camera.exposure_mode='off'
camera.awb_mode='off'
#g = camera.awb_gains
camera.digital_gain=1
camera.awb_gains=0.9
totalFrames = 3
i = 0

print("[INFO] configuring read pin and event trigger")
GPIO.setmode(GPIO.BCM)
IN_PIN = 18
OUT_PIN = 27
GPIO.setup(OUT_PIN, GPIO.OUT)
GPIO.setup(IN_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

print("[INFO] waiting trigger")
while i < totalFrames:
    start = time.time_ns()
    GPIO.wait_for_edge(IN_PIN, GPIO.RISING)
    camera.capture('foo.jpg')
    print('\n[INFO] img captured    - ', (time.time_ns()-start)/(10**9))
    GPIO.output(OUT_PIN,GPIO.HIGH)
    print('[INFO] ack sent        - ', (time.time_ns()-start)/(10**9))
    GPIO.wait_for_edge(IN_PIN, GPIO.FALLING)
    print('[INFO] trigger down    - ', (time.time_ns()-start)/(10**9))
    GPIO.output(OUT_PIN,GPIO.LOW)
    print('[INFO] loop finished   - ', (time.time_ns()-start)/(10**9))
    i+=1

camera.stop_preview()
GPIO.cleanup()
frame = cv2.imread('foo.jpg')
frame = cv2.cvtColor(frame,cv2.COLOR_RGB2GRAY)
cv2.imshow("Frame", frame)
cv2.waitKey(0)
cv2.destroyAllWindows()


