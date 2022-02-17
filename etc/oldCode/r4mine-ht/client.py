import socket
import time 

serverAddressPort   = ("192.168.0.103", 8888)
bufferSize          = 1024

# Create a UDP socket at client side
UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

# Send to server using created UDP socket
timeMsg = str.encode(str(time.time_ns()))
UDPClientSocket.sendto(timeMsg, serverAddressPort)
msgFromServer = UDPClientSocket.recvfrom(bufferSize)

msg = " >> " + str((int(msgFromServer[0])-int(timeMsg))/(10 ** 9)) + 'ns'
print(msg)
print(msgFromServer[1])