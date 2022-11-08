#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32MultiArray.h>
// #include <wittenstein_msgs/wittenstein.h>
#include <rosbag/bag.h>
#include <xela_server/xServerMsg.h>

// Client side C/C++ program to demonstrate Socket
// programming
#include <arpa/inet.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#define PORT 50001
//rostopic echo -b impulse_coll.bag -p /write > ~/Desktop/imp_test.csv

#include <string>
#include <cstring>
#include <sstream>
#include <iostream>

using namespace std;

xela_server::xServerMsg global_msg2;

int xela, xela1, xela2, xela3, xela4, 
	xela5, xela6, xela7, xela8, xela9,
	xela10, xela11, xela12, xela13, xela14, xela15;

void callback2(const xela_server::xServerMsg::ConstPtr& msg2){

	global_msg2.points = msg2->points;
	xela = global_msg2.points[0].point.z;
	xela1 = global_msg2.points[1].point.z;
	xela2 = global_msg2.points[2].point.z;
	xela3 = global_msg2.points[3].point.z;
	xela4 = global_msg2.points[4].point.z;
	xela5 = global_msg2.points[5].point.z;
	xela6 = global_msg2.points[6].point.z;
	xela7 = global_msg2.points[7].point.z;
	xela8 = global_msg2.points[8].point.z;
	xela9 = global_msg2.points[9].point.z;
	xela10 = global_msg2.points[10].point.z;
	xela11 = global_msg2.points[11].point.z;
	xela12 = global_msg2.points[12].point.z;
	xela13 = global_msg2.points[13].point.z;
	xela14 = global_msg2.points[14].point.z;
	xela15 = global_msg2.points[15].point.z;
	
    // std::cout<< xela <<std::endl;	
}

std::string to_string(int x)
{
	std::ostringstream ss;
	ss << x;
	return ss.str();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "listener_global");
	ros::NodeHandle n;
	ros::Rate rate(100); // 500Hz -> 0.002s
	int count = 0;

	// ros::Subscriber sub_wittenstein = n.subscribe("wittenstein_topic", 1000, &callback);
	// ros::Subscriber sub_xela = n.subscribe("xServerPub", 1000, &callback2);
	ros::Subscriber sub_xela = n.subscribe("xServTopic", 1000, &callback2);
	std::string bag_name = "xela_calibration.bag";
	rosbag::Bag bag("/home/elonmuskswife/catkin_ws/src/xela_server/bagfiles/" + bag_name, rosbag::bagmode::Write);


	// SOCKET PROGRAMMING
	int sock = 0, valread, client_fd;
    struct sockaddr_in serv_addr;
    char buffer[1024] = { 0 };
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        printf("\n Socket creation error \n");
        return -1;
    }
 
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PORT);
 
    // Convert IPv4 and IPv6 addresses from text to binary form
    if (inet_pton(AF_INET, "192.168.0.101", &serv_addr.sin_addr)
        <= 0) {
        printf(
            "\nInvalid address/ Address not supported \n");
        return -1;
    }
    if ((client_fd
         = connect(sock, (struct sockaddr*)&serv_addr,
                   sizeof(serv_addr)))
        < 0) {
        printf("\nConnection Failed \n");
        return -1;
    }
	// SOCKET PROGRAMMING END 
	
	string hello_var;

	while(ros::ok()){

		std_msgs::Float32MultiArray data_out;
		data_out.data.resize(2);
        // data_out.data[0] = global_msg2.points[1].point.z;
		bag.write("write", ros::Time::now(), data_out);

		char* hello = "Hello from client";
		// double pi = xela;
  		// std::string str = to_string(pi);
		// std::cout << str << std::endl;
		
		// char* hello = const_cast<char*>std::to_string(xela);
		// int number_to_send = xela;
		// int converted_num = htonl(number_to_send);


		// char* hello = to_string(xela);
   		send(sock, hello, strlen(hello), 0);
		// write(sock, &converted_num, sizeof(converted_num));
    	printf("xela message sent\n");
    	valread = read(sock, buffer, 1024);
   	 	printf("%s\n", buffer);
		//ROS_INFO_STREAM("Processing");
		ros::spinOnce();
		rate.sleep();
	}
	bag.close();

	// closing the connected socket
    close(client_fd);
    return 0;
}

/*header: 
  seq: 2620
  stamp: 
    secs: 1667796391
    nsecs: 333096504
  frame_id: ''
sensor: 1
model: "XR1944"
points: 
  - 
    taxels: 0
    point: 
      x: 16881.0
      y: 16539.0
      z: 36839.0*/