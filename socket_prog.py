#!/usr/bin/env python3
# SERVER HERE
import queue
import rospy
import socket
import sys
from std_msgs.msg import Int64, Float32MultiArray, Bool
# from xela_server.msg import xServerMsg
from xela_server.msg import xServerMsg, xSensorData
# import keyboard 

class Nodo():
    def __init__(self):

        sub = rospy.Subscriber('xServerPub',xServerMsg, self.callback)
        self.taxs = 0
        self.xelax = 0
        self.xelay = 0
        self.xelaz = 0

        rospy.spin()
        rate = rospy.Rate(10) #10Hz

        while not rospy.is_shutdown():
            # rospy.loginfo(self.xela0)
            rate.sleep()

    def xyz(self,data):
        self.xelax = data.xyz.x
        self.xelay = data.xyz.y
        self.xelaz = data.xyz.z
        print(self.taxs)
        print(self.xelax)
        print(self.xelay)
        print(self.xelaz)

    def callback(self, data): # handling of the xela messages
        
        self.taxs = data.taxel
        
        if(self.taxs == 1):
            self.xyz(data)
        elif(self.taxs == 2):
            self.xyz(data)
        elif(self.taxs == 3):
            self.xyz(data)
        elif(self.taxs == 4):
            self.xyz(data)
        elif(self.taxs == 5):
            self.xyz(data)
        elif(self.taxs == 6):
            self.xyz(data)
        elif(self.taxs == 7):
            self.xyz(data)
        elif(self.taxs == 8):
            self.xyz(data)
        elif(self.taxs == 9):
            self.xyz(data)
        elif(self.taxs == 10):
            self.xyz(data)
        elif(self.taxs == 11):
            self.xyz(data)
        elif(self.taxs == 12):
            self.xyz(data)
        elif(self.taxs == 13):
            self.xyz(data)
        elif(self.taxs == 14):
            self.xyz(data)
        elif(self.taxs == 15):
            self.xyz(data)
        elif(self.taxs == 16):
            self.xyz(data)
            
        # toSend = str(int(self.xela0)) + '|' + str(int(self.xela1)) + '|' + str(int(self.xela2)) + '|' + str(int(self.xela3)) + '|' + str(int(self.xela4)) + '|' + str(int(self.xela5)) + '|'
        # toSend = toSend + str(int(self.xela6)) + '|' + str(int(self.xela7)) + '|' + str(int(self.xela8)) + '|' + str(int(self.xela9)) + '|' + str(int(self.xela10)) + '|' + str(int(self.xela11)) + '|'
        # toSend = toSend + str(int(self.xela12)) + '|' + str(int(self.xela13)) + '|' + str(int(self.xela14)) + '|' + str(int(self.xela15)) + ']'
        # print(toSend)
        # try: 
        #     connection.send(str(toSend).encode('utf8')) # sending information 

        # except KeyboardInterrupt:
        #     print("Closing")
        #     connection.close()
        #     reason="had to  "
        #     rospy.signal_shutdown(reason)
        #     exit()

        # print(keyboard.read_key())

        # if keyboard.read_key() == "q":
        # # if keyboard.is_pressed('q'):
        #     print("Closing")
        #     connection.close()
        #     reason="had to  "
        #     rospy.signal_shutdown(reason)
        #     exit()       
        # try:  # used try so that if user pressed other than the given key error will not be shown
        #     if keyboard.is_pressed('q'):  # if key 'q' is pressed 
        #         print('You Pressed A Key!')
        #         print("Closing")
        #         connection.close()
        #         reason="had to  "
        #         rospy.signal_shutdown(reason)
        #         exit()# finishing the loop
        # except:
        # #     # if user pressed a key other than the given key the loop will break


if __name__=='__main__':

    # Create a TCP/IP socket
    # s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # Bind the socket to the port
    # server_address = ("192.168.0.100", 50009)
    # print('starting up on {} port {}'.format(*server_address))
    # s.bind(server_address)

    # Listen for incoming connections
    # s.listen()
    # Wait for a connection
    # print('waiting for a connection')
    # connection, client_address = s.accept()
    # print(sys.stderr, 'connection from', client_address)

    # with connection:
        # print(f"Connected by {client_address}")
        # while True:
        #     # try:
        #     #     toSend = str(35064) + '|' + str(21654) 
        #     #     connection.send(str(toSend).encode('utf8')) # sending information 
        #     # except:
        #     #     print("Closing")
        #     #     connection.close()
        # rospy.init_node('talker', anonymous=True)
        # new = Nodo()
    rospy.init_node('talker', anonymous=True)
    new = Nodo()

# connection.close()





# import socket
# HOST = "192.168.0.101"  # The server's hostname or IP address
# PORT = 50000  # The port used by the server

# with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
#     s.connect((HOST, PORT))
#     s.sendall(b"Hello, world")
#     data = s.recv(1024)

# print(f"Received {data!r}")
