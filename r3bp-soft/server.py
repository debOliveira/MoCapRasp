import socket
import time
from picamera import PiCamera
import cv2

print("[INFO] setting camera")
camera = PiCamera()
camera.resolution = (640,480)
camera.sensor_mode = 7
camera.framerate = 60
time.sleep(2)
camera.shutter_speed = camera.exposure_speed
camera.iso = 100
camera.exposure_mode = 'off'
g = camera.awb_gains
camera.awb_mode = 'off'
camera.awb_gains = g


localIp = "192.168.1.103"
localPort = 8888
bufferSize = 1024
UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
UDPServerSocket.bind((localIp,localPort))
print("[INFO] server running...")

bytesPair = UDPServerSocket.recvfrom(bufferSize)
adress = bytesPair[1]
timeBase = time.time_ns()
print(str(adress) + ' >> '+ str(int(timeBase)/(10**9)))
UDPServerSocket.sendto(str.encode(str(timeBase)),adress)

now = time.time_ns()
while (now - timeBase) < 5000000:
    now = time.time_ns()
    
print((now - timeBase)/(10**9),now/(10**9))
timeBase += 5000000

while (now - timeBase) < 5000000:
    now = time.time_ns()
    
print((now - timeBase)/(10**9),now/(10**9))
timeBase += 5000000

while (now - timeBase) < 5000000:
    now = time.time_ns()

print((now - timeBase)/(10**9),now/(10**9))
camera.capture('foo.jpg')
frame = cv2.imread('foo.jpg')
frame = cv2.cvtColor(frame,cv2.COLOR_RGB2GRAY)
cv2.imshow(f"Frame",frame)
cv2.waitKey(0)
cv2.destroyAllWindows()


