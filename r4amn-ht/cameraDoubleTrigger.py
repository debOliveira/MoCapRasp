import RPi.GPIO as GPIO
import time
from picamera import PiCamera
import cv2
import time
    
print("[INFO] init `picamera` module...")
vs = PiCamera()
vs.sensor_mode = 7
vs.resolution = (640,480)
vs.framerate = 90
vs.awb_mode='off'
vs.awb_gains=1.6
vs.start_preview(fullscreen=False)
time.sleep(1)

def trigger_callback(IN_PIN):
    global frame    
    vs.capture('foo.jpg')
    print("[INFO] camera triggered!")
    '''GPIO.output(OUT_PIN, GPIO.HIGH)
    time.sleep(0.002)
    GPIO.output(OUT_PIN, GPIO.LOW)
    time.sleep(0.002)'''

print("[INFO] configuring read pin and event trigger")
IN_PIN = 18
OUT_PIN = 27
GPIO.setmode(GPIO.BCM)
GPIO.setup(IN_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(OUT_PIN,GPIO.OUT)
vs.exposure_mode='off'
GPIO.add_event_detect(IN_PIN, GPIO.RISING, callback=trigger_callback)

print("[INFO] waiting trigger")
while True:
    try:
        frame = cv2.imread('foo.jpg')
        cv2.imshow("Frame", frame)
        cv2.waitKey(1)
    except KeyboardInterrupt:
        GPIO.cleanup()
        break
vs.stop_preview()
cv2.destroyAllWindows()
