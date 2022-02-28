import socket
import numpy as np

bufferSize = 1024
server_socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
server_socket.bind(('0.0.0.0',8888))
print("[INFO] server running, waiting...")

while True:
    bytesPair = server_socket.recvfrom(bufferSize)
    message = np.frombuffer(bytesPair[0])
    if not (len(message)-1):
        break
    address = bytesPair[1]

    clientMsg = "Message from Client:{}".format(message)
    clientIP  = "Client IP Address:{}".format(address)
    
    print(clientMsg)
    print(clientIP)

server_socket.close()