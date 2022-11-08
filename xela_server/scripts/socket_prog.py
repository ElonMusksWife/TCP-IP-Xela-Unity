#!/usr/bin/env python3
# SERVER HERE
import queue
import rospy
import socket
import sys
from std_msgs.msg import Int64, Float32MultiArray, Bool
from xela_server.msg import xServerMsg

class Nodo():
    def __init__(self):

        sub = rospy.Subscriber('xServTopic',xServerMsg, self.callback)
        self.xela0 = 0
        self.xela1 = 0
        self.xela2 = 0
        self.xela3 = 0
        self.xela4 = 0
        self.xela5 = 0
        self.xela6 = 0
        self.xela7 = 0
        self.xela8 = 0
        self.xela9 = 0
        self.xela10 = 0
        self.xela11 = 0
        self.xela12 = 0
        self.xela13 = 0
        self.xela14 = 0
        self.xela15 = 0
        rospy.spin()
        rate = rospy.Rate(10) #10Hz

        while not rospy.is_shutdown():
            # rospy.loginfo(self.xela0)
            rate.sleep()

    def callback(self, data): # handling of the xela messages

        self.xela0 = data.points[0].point.z
        self.xela1 = data.points[1].point.z
        self.xela2 = data.points[2].point.z
        self.xela3 = data.points[3].point.z
        self.xela4 = data.points[4].point.z
        self.xela5 = data.points[5].point.z
        self.xela6 = data.points[6].point.z
        self.xela7 = data.points[7].point.z
        self.xela8 = data.points[8].point.z
        self.xela9 = data.points[9].point.z
        self.xela10 = data.points[10].point.z
        self.xela11 = data.points[11].point.z
        self.xela12 = data.points[12].point.z
        self.xela13 = data.points[13].point.z
        self.xela14 = data.points[14].point.z
        self.xela15 = data.points[15].point.z

        toSend = str(self.xela0) + '|' + str(self.xela1) + '|' + str(self.xela2) + '|' + str(self.xela3) + '|' + str(self.xela4) + '|' + str(self.xela5) + '|'
        toSend = toSend + str(self.xela6) + '|' + str(self.xela7) + '|' + str(self.xela8) + '|' + str(self.xela9) + '|' + str(self.xela10) + '|' + str(self.xela11) + '|'
        toSend = toSend + str(self.xela12) + '|' + str(self.xela13) + '|' + str(self.xela14) + '|' + str(self.xela15) + '|'

        try: 
            connection.send(str(toSend).encode('utf8')) # sending information 
        except:
            print("Closing")
            connection.close()
            reason="had to  "
            rospy.signal_shutdown(reason)
            exit()


if __name__=='__main__':

    # Create a TCP/IP socket
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # Bind the socket to the port
    server_address = ("192.168.0.100", 50002)
    print('starting up on {} port {}'.format(*server_address))
    s.bind(server_address)

    # Listen for incoming connections
    s.listen()
    # Wait for a connection
    print('waiting for a connection')
    connection, client_address = s.accept()
    print(sys.stderr, 'connection from', client_address)

    with connection:
        print(f"Connected by {client_address}")
        # while True:
        #     # try:
        #     #     toSend = str(35064) + '|' + str(21654) 
        #     #     connection.send(str(toSend).encode('utf8')) # sending information 
        #     # except:
        #     #     print("Closing")
        #     #     connection.close()
        rospy.init_node('talker', anonymous=True)
        new = Nodo()

connection.close()


# import socket
# HOST = "192.168.0.101"  # The server's hostname or IP address
# PORT = 50000  # The port used by the server

# with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
#     s.connect((HOST, PORT))
#     s.sendall(b"Hello, world")
#     data = s.recv(1024)

# print(f"Received {data!r}")
