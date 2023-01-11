#!/usr/bin/env python
import queue
import rospy
import socket
import sys
import os
import json
from std_msgs.msg import Int64, Float32MultiArray, Bool, Float32
# from xela_server.msg import xServerMsg
from xela_server.msg import xServerMsg, xSensorData
# import keyboard 
from sensor_msgs.msg import JointState

# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
# Bind the socket to the port
server_address = ("10.1.71.40", 50000)
print(sys.stderr, "starting up on %s port %s" % server_address)

sock.bind(server_address)

# Listen for incoming connections
sock.listen(1)

print(sys.stderr, "waiting for a connection")
connection, client_address = sock.accept()
print(sys.stderr, "connection from", client_address)

class Nodo():

    def __init__(self):

        #  variables
        self.taxs = 0
        self.xelax = 0
        self.xelay = 0
        self.xelaz = 0

        # self.xela1 = None
        # self.xela2 = None
        # self.xela3 = None
        # self.xela4 = None
        # self.xela5 = None
        # self.xela6 = None
        # self.xela7 = None
        # self.xela8 = None
        # self.xela9 = None
        # self.xela10 = None
        # self.xela11 = None
        # self.xela12 = None
        # self.xela13 = None
        # self.xela14 = None
        # self.xela15 = None
        # self.xela16 = None

        self.xela = None
        self.pos = None
        self.xbinded = None
        # subscribers
        sub = rospy.Subscriber('xServerPub',xServerMsg, self.callback1)
        sub_collect = rospy.Subscriber('/chatter',Float32, self.simple_callback)
        sub_franka = rospy.Subscriber('/franka_state_controller/joint_states', JointState, self.callback)

        rospy.spin()
        rate = rospy.Rate(10) #10Hz

    def simple_callback(self, msg):
        self.collection = msg.data

    def xyz(self,data):
        self.xelax = data.xyz.x
        self.xelay = data.xyz.y
        self.xelaz = data.xyz.z

        self.xbinded = [self.xelax, self.xelay, self.xelaz]

        
    def callback1(self, data): # handling of the xela messages
        
        self.taxs = data.taxel
        
        if(self.taxs == 1):
            self.xyz(data)
            self.xela = [self.xelax, self.xelay, self.xelaz]
            # self.xela1 = self.xela
        elif(self.taxs == 2):
            self.xyz(data)
            # self.xela2 = self.xela
            self.xela.extend(self.xbinded)
        elif(self.taxs == 3):
            self.xyz(data)
            # self.xela3 = self.xela
            self.xela.extend(self.xbinded)
        elif(self.taxs == 4):
            self.xyz(data)
            # self.xela4 = self.xela
            self.xela.extend(self.xbinded)
        elif(self.taxs == 5):
            self.xyz(data)
            # self.xela5 = self.xela
            self.xela.extend(self.xbinded)
        elif(self.taxs == 6):
            self.xyz(data)
            # self.xela6 = self.xela
            self.xela.extend(self.xbinded)
        elif(self.taxs == 7):
            self.xyz(data)
            # self.xela7 = self.xela
            self.xela.extend(self.xbinded)
        elif(self.taxs == 8):
            self.xyz(data)
            # self.xela8 = self.xela
            self.xela.extend(self.xbinded)
        elif(self.taxs == 9):
            self.xyz(data)
            # self.xela9 = self.xela
            self.xela.extend(self.xbinded)
        elif(self.taxs == 10):
            self.xyz(data)
            # self.xela10 = self.xela
            self.xela.extend(self.xbinded)
        elif(self.taxs == 11):
            self.xyz(data)
            # self.xela11 = self.xela
            self.xela.extend(self.xbinded)
        elif(self.taxs == 12):
            self.xyz(data)
            # self.xela12 = self.xela
            self.xela.extend(self.xbinded)
        elif(self.taxs == 13):
            self.xyz(data)
            # self.xela13 = self.xela
            self.xela.extend(self.xbinded)
        elif(self.taxs == 14):
            self.xyz(data)
            # self.xela14 = self.xela
            self.xela.extend(self.xbinded)
        elif(self.taxs == 15):
            self.xyz(data)
            # self.xela15 = self.xela
            self.xela.extend(self.xbinded)
        elif(self.taxs == 16):
            self.xyz(data)
            # self.xela16 = self.xela
            self.xela.extend(self.xbinded)
    
    
    def callback(self, data):
        self.pos = data.position
        self.pos = list(self.pos)
        self.pos.extend(self.xela)

        self.pos = tuple(self.pos)
        b = json.dumps(self.pos)

        print(self.xela)

        try:
            connection.send(b.encode())
            print(b)

        except:
            print("closing connection")
            connection.close()
            reason = "i need to"
            rospy.signal_shutdown(reason)
            exit()


if __name__=='__main__':
    rospy.init_node('wearami_socket', anonymous=True)
    s = Nodo()

connection.close()
