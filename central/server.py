import io
import socket
import struct
from PIL import Image
import cv2
import numpy as np
import argparse

# argument parser configuration
msg = "This function opens the stream for receiving the images"
parser = argparse.ArgumentParser(description = msg)
parser.add_argument("-c", "--camera", help = "Number X of the camera that should be listened")
args = parser.parse_args()
number = int(args.camera)

# init server
print('[INFO] starting server '+str(number))
server_socket = socket.socket()
server_socket.bind(('0.0.0.0', 7999+number))
server_socket.listen(0)
count = 0
imgBG = cv2.imread("../results/camera"+str(number)+".jpg", cv2.IMREAD_GRAYSCALE)


# Accept a single connection and make a file-like object out of it
connection = server_socket.accept()[0].makefile('rb')
print('[INFO] client connected')
try:
    while True:
        # Read the length of the image as a 32-bit unsigned int. If the
        # length is zero, quit the loop
        image_len = struct.unpack('<L', connection.read(struct.calcsize('<L')))[0]
        if not image_len:
            break
        # Construct a stream to hold the image data and read the image
        # data from the connection
        image_stream = io.BytesIO()
        image_stream.write(connection.read(image_len))
        # Rewind the stream, open it as an image with PIL and do some
        # processing on it
        image_stream.seek(0)
        image = Image.open(image_stream)
        # subtract clean plate
        img = cv2.cvtColor(np.array(image),cv2.COLOR_RGB2GRAY)
        imgMarkers = cv2.subtract(img, imgBG) 
        # save image
        image = Image.fromarray(imgMarkers)
        image.save('../results/raw/camera'+str(number)+'/'+str(count).zfill(4)+'.jpg')
        #image.verify()
        #print('Image is verified')
        count+=1
finally:
    connection.close()
    server_socket.close()
    print('[FINISHED]')
