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
camera.iso = 10
camera.start_preview(fullscreen=False)
time.sleep(10)
camera.shutter_speed = camera.exposure_speed
camera.exposure_mode='off'
g = camera.awb_gains
camera.awb_mode = 'off'
#camera.analog_gain=2
camera.awb_gains = g

def ack_callback(IN_PIN):
    '''global isReady
    isReady = True'''
    print("[INFO] ack received")
    
print("[INFO] setting pins...")
GPIO.setmode(GPIO.BCM)
OUT_PIN = 18
IN_PIN = 27
GPIO.setup(OUT_PIN, GPIO.OUT)
GPIO.setup(IN_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.add_event_detect(IN_PIN, GPIO.RISING, callback=ack_callback)
i = 0
totalFrames = 1
#isReady = True

while i < totalFrames:
    try:
        '''if isReady:
            i+=1
            #isReady = False
            print("[INFO-"+str(i-1).zfill(2)+"] sending trigger...")
            GPIO.output(OUT_PIN, GPIO.HIGH)
            camera.capture('foo.jpg')
            time.sleep(0.008)
            print("[INFO-"+str(i-1).zfill(2)+"] going down...")
            GPIO.output(OUT_PIN, GPIO.LOW)
            time.sleep(0.008)
            print("[INFO-"+str(i-1).zfill(2)+"] restarting...")'''
    except KeyboardInterrupt:
        print("Aborted")
        break
    

print("[INFO] turning off...")
GPIO.cleanup()
camera.stop_preview()

print('Analog gain:',camera.analog_gain)
print('Digital gain:',camera.analog_gain)
print('AWB gain:',camera.awb_gains)
print('Exposure speed (micros):',camera.exposure_speed)
print('Sensor mode:',camera.sensor_mode)
print('Framerate delta:',camera.framerate_delta)

frame = cv2.imread('foo.jpg')
cv2.imshow('Frame',frame)
cv2.waitKey(0)
cv2.destroyAllWindows()