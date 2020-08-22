#include "ros/ros.h"
#include "sensor_msgs/FluidPressure.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <arpa/inet.h>
#include <unistd.h>
#define SERVER_IP_1 "118.178.180.70"
#define SERVPORT 9000
char buffer[100];
int sockfd_1;
struct sockaddr_in serv_addr_1;

void pressureCallback(const sensor_msgs::FluidPressure::ConstPtr& msg)
{
    sprintf(buffer,"$%f,%f\n",msg->fluid_pressure,msg->variance);
     // socket transmit
    write(sockfd_1, buffer,strlen(buffer));
    ROS_INFO("I heard: [%s]",buffer);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "pressure_pub");
  ros::NodeHandle n;
  ros::Subscriber sub;// = n.subscribe("/pressure", 100, pressureCallback);
  // socket init 
	if ((sockfd_1 = socket(AF_INET, SOCK_STREAM, 0)) == -1)
		{
			ROS_INFO("socket error!");
			return -1;
		}
	bzero(&serv_addr_1, sizeof(serv_addr_1));
	serv_addr_1.sin_family = AF_INET;
	serv_addr_1.sin_port = htons(SERVPORT);
	serv_addr_1.sin_addr.s_addr = inet_addr(SERVER_IP_1);
	connect(sockfd_1, (struct sockaddr *) &serv_addr_1, sizeof(struct sockaddr));
	sub = n.subscribe("/pressure", 100, pressureCallback);
	ros::spin();
	close(sockfd_1);
	return 0;
}