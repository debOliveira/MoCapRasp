import RPi.GPIO as GPIO
import time
import picamera
import threading

def button_callback(channel):
    print("Button was pushed!")

camera = picamera.PiCamera()
camera.resolution = (640, 480)

IN_PIN = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(IN_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) 
GPIO.add_event_detect(IN_PIN,GPIO.RISING,callback=button_callback)

message = input("Press enter to quit\n")
GPIO.cleanup() 
