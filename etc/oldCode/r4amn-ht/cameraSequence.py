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
camera.iso = 10
camera.start_preview(fullscreen=False)
time.sleep(10)
camera.shutter_speed=camera.exposure_speed
camera.exposure_mode='off'
camera.awb_mode='off'
g = camera.awb_gains
#camera.digital_gain=2
camera.awb_gains=g
camera.capture('foo.jpg')

def trigger_callback(IN_PIN):
    camera.capture('foo.jpg')
    global inCallbackFlag
    inCallbackFlag = True
    print("[INFO] TRIGGERED")
    GPIO.output(OUT_PIN, GPIO.HIGH)
    time.sleep(0.001)
    inCallbackFlag = False

print("[INFO] configuring read pin and event trigger")
GPIO.setmode(GPIO.BCM)
IN_PIN = 18
OUT_PIN = 27
GPIO.setup(OUT_PIN, GPIO.OUT)
GPIO.setup(IN_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.add_event_detect(IN_PIN,GPIO.RISING,callback=trigger_callback)
inCallbackFlag = False

print("[INFO] waiting trigger")
while True:
    try:
        if not inCallbackFlag:
            GPIO.output(OUT_PIN, GPIO.LOW)
        '''for i in range(0,10):
            GPIO.wait_for_edge(IN_PIN, GPIO.RISING)
            print("[INFO-"+str(i).zfill(2)+"] camera triggered!")
            #GPIO.output(OUT_PIN, GPIO.HIGH)
            camera.capture('foo.jpg')
            #time.sleep(0.001)
            #GPIO.output(OUT_PIN, GPIO.LOW)
            #time.sleep(0.001)
            #print("[INFO-"+str(i).zfill(2)+"] ack sent!")'''
    except KeyboardInterrupt:
        print("Aborted")
        break
    
camera.stop_preview()
GPIO.cleanup()
frame = cv2.imread('foo.jpg')
frame = cv2.cvtColor(frame,cv2.COLOR_RGB2GRAY)
cv2.imshow("Frame", frame)
cv2.waitKey(0)
cv2.destroyAllWindows()

