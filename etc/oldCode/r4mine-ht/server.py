import socket
import time 

localIP     = "192.168.0.102"
localPort   = 8888
bufferSize  = 1024

# Create a datagram socket
UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM) 

# Bind to address and ip
UDPServerSocket.bind((localIP, localPort))

print("UDP server up and listening")

# Listen for incoming datagrams
while(True):
    bytesAddressPair = UDPServerSocket.recvfrom(bufferSize)
    message = bytesAddressPair[0]
    address = bytesAddressPair[1]
    clientIP  = "Client IP Address:{}".format(address) 
    print(clientIP) 
    
    timeMsg = str.encode(str(time.time_ns()))
    print(clientIP + ' >> ' + str((int(timeMsg) - int(message))/(10 ** 9)) + 's')
    
    # Sending a reply to client
    UDPServerSocket.sendto(timeMsg, address)
