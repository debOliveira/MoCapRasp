import RPi.GPIO as GPIO
from picamera import PiCamera
import cv2
import time
    
print("[INFO] init `picamera` module...")
vs = PiCamera()
vs.resolution=(640,480)
vs.start_preview(fullscreen=False)
time.sleep(2.0)
vs.capture('foo.jpg')
def ack_callback(IN_PIN): 
    print("[INFO] ack received")

def send_trigger():
    global frame
    GPIO.output(OUT_PIN, GPIO.HIGH)
    time.sleep(0.005)
    vs.capture('foo.jpg')
    GPIO.output(OUT_PIN, GPIO.LOW)
    time.sleep(0.005)
    
print("[INFO] configuring pins...")
OUT_PIN = 18
IN_PIN = 27
GPIO.setmode(GPIO.BCM)
GPIO.setup(OUT_PIN, GPIO.OUT)
GPIO.setup(IN_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.add_event_detect(IN_PIN, GPIO.RISING, callback=ack_callback)

print("[INFO] sending trigger (press enter)")
i = 0
while i < 10:
    try:
        send_trigger()
        time.sleep(.01)
        i+=1
    except KeyboardInterrupt:
        GPIO.cleanup()
        break

vs.stop_preview()