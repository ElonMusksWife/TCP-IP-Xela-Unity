#!/usr/bin/env python3
import socket
#Socket initialized here !!!!!!!!!!!!!!!!!!!----------------
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Connect the socket to the port where the server is listening
server_address = ("192.168.0.100", 50002) 
print('connecting to {} port {}'.format(*server_address))
sock.connect(server_address)
    
while True:

    #Socket data recieved here !!!!!!!!!!!!--------------------
    data = sock.recv(1024) 
    # data = data
    incomingData = data.decode("utf-8").split("|")
    # HERE incomingData[0] is FSR data mapped 0-255
    #      incomingData[1] is RobotLOWstate True or False

    # num3 = int(float(int(incomingData[0])))
    xela0 = int(float(incomingData[0]))
    xela1 = int(float(incomingData[1]))
    xela2 = int(float(incomingData[2]))
    xela3 = int(float(incomingData[3]))
    xela4 = int(float(incomingData[4]))
    xela5 = int(float(incomingData[5]))
    xela6 = int(float(incomingData[6]))
    xela7 = int(float(incomingData[7]))
    xela8 = int(float(incomingData[8]))
    xela9 = int(float(incomingData[9]))
    xela10 = int(float(incomingData[10]))
    xela11 = int(float(incomingData[11]))
    xela12 = int(float(incomingData[12]))
    xela13 = int(float(incomingData[13]))
    xela14 = int(float(incomingData[14]))
    xela15 = int(float(incomingData[15]))


    #Socket!!!!!!!!!!!!!!!!-------------
    # toSend = str(num) + '|' + str(ds.state.cross) + '|' + str(ds.state.triangle) #+ '|' + str(ds.state.LX) + '|' + str(ds.state.LY)
    # sock.send(str(toSend).encode('utf8')) # L2 trigger state is sent from here 
    print (*["Taxels values are:",xela0,xela1,xela2,xela3,xela4,xela5, xela6,xela7,xela8,xela9,xela10,xela11,xela12,xela13,xela14,xela15])

    if not data:
        break
