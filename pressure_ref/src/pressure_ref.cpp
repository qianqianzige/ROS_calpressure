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
#include <sstream> //使用stringstream需要引入这个头文件
using namespace std;
#define SERVER_IP_1 "118.178.180.70"
#define SERVPORT 9001
char buffer[100];
int sockfd_1;
struct sockaddr_in serv_addr_1;
char   pressure_temp[2][12];
sensor_msgs::FluidPressure msg;
ros::Publisher pub;
void  pressureScanf(const char *buff)
{
	unsigned short i = 1;
    unsigned char j = 0;
    for (; buff[i] != '\0'; ++i)
    {
        if (buff[i] == ',')
            ++j;
        if (j == 1)
        {
            sscanf(buff, "%[^,],%[^*]*",pressure_temp[0], pressure_temp[1]);
            istringstream iss(pressure_temp[0]);
			iss >> msg.fluid_pressure;
            istringstream iss2(pressure_temp[1]);
			iss2 >> msg.variance;
            ROS_INFO_STREAM("pressure:"<<  msg.fluid_pressure<<"temp:" <<msg.variance);
            pub.publish(msg);
            break;
        }
    }
}



void PressureRefParse(const char *buff, const short len)
{
    unsigned short j = 0;
    while (j < len)
    {
        if (buff[j] == '$')
        {
        	pressureScanf(buff+1);
        }
        ++j;
    }
}




void pressureDecode(const char * buff)
{
   

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "pressure_ref");
  ros::NodeHandle n;
  pub = n.advertise<sensor_msgs::FluidPressure>("/pressure", 1000);
  int rval;
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
   	ros::Rate loop_rate(10);
    while(ros::ok())
    {
    	if ((rval = read(sockfd_1, buffer, 100)) < 0)
    	{
			ROS_INFO("reading stream error!");
			ros::spinOnce();
			loop_rate.sleep();
			continue;
		}
		if (rval>0)
		{
			// decode 
			ROS_INFO_STREAM(buffer);
			PressureRefParse(buffer,rval);
		}

		ros::spinOnce();
		loop_rate.sleep();
    }

	ros::spin();
	close(sockfd_1);
	return 0;
}