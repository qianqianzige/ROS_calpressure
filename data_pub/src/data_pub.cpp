#include "ros/ros.h"
#include "sensor_msgs/Imu.h" //未确定
#include "sensor_msgs/FluidPressure.h"
#include <sensor_msgs/NavSatFix.h>
#include <math.h>
# include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <ctime>
#include <iostream>
#include <sstream>
using namespace std;
#define SERVER_IP_1 "118.178.180.70" //需修改
#define SERVPORT 7002 //需修改

double a = 6378137;         // earth radius in meters 
double b = 6356752.3142;    // earth semiminor in meters
double f = (a - b)/a;
double e_sq = f * (2-f);
# define pi     3.14159265359
# define rat_dgr_to_rad  1.0/180.0*pi
//#define SERVER_IP_1 "118.178.180.70" //需修改
//#define SERVPORT 9000 //需修改
char buffer[1000];
int sockfd_1;
struct sockaddr_in serv_addr_1;
nav_msgs::Odometry msg_ecef;

nav_msgs::Odometry enu_to_ecef(nav_msgs::Odometry msg_enu, nav_msgs::Odometry msg_wgs0)
{
    double xEast=msg_enu.pose.pose.position.x, yNorth=msg_enu.pose.pose.position.y, zUp=msg_enu.pose.pose.position.z, 
    lat0=msg_wgs0.pose.pose.position.x, lon0=msg_wgs0.pose.pose.position.y, h0=msg_wgs0.pose.pose.position.z;

    double lamb = rat_dgr_to_rad  * lat0;
    double phi = rat_dgr_to_rad  * lon0;
    double s = sin(lamb);
    double N = a / sqrt(1 - e_sq * s * s);

    double sin_lambda = sin(lamb);
    double cos_lambda = cos(lamb);
    double sin_phi = sin(phi);
    double cos_phi = cos(phi);

    double x0 = (h0 + N) * cos_lambda * cos_phi;
    double y0 = (h0 + N) * cos_lambda * sin_phi;
    double z0 = (h0 + (1 - e_sq) * N) * sin_lambda;

    double t = cos_lambda * zUp - sin_lambda * yNorth;

    double zd = sin_lambda * zUp + cos_lambda * yNorth;
    double xd = cos_phi * t - sin_phi * xEast; 
    double yd = sin_phi * t + cos_phi * xEast;

    double x = xd + x0 ;
    double y = yd + y0 ;
    double z = zd + z0 ;

    nav_msgs::Odometry msg_ecef = msg_enu;
    nav_msgs::Odometry msg_gps;
    msg_ecef.pose.pose.position.x  =x;
    msg_ecef.pose.pose.position.y  =y;
    msg_ecef.pose.pose.position.z  =z;
    return msg_ecef;
}

nav_msgs::Odometry ecef_to_geodetic(nav_msgs::Odometry msg_ecef)
{
    // Convert from ECEF cartesian coordinates to 
    // latitude, longitude and height.  WGS-84
    double x=msg_ecef.pose.pose.position.x, y=msg_ecef.pose.pose.position.y,
    z=msg_ecef.pose.pose.position.z;

    double x2 = x * x ;
    double y2 = y * y ;
    double z2 = z * z ;
   
    double e = sqrt (1-pow((b/a),2)) ;
    double b2 = b*b ;
    double e2 = e *e ;
    double ep = e*(a/b) ;
    double r = sqrt(x2+y2) ;
    double r2 = r*r ;
    double E2 = a *a - b*b ;
    double F = 54*b2*z2 ;
    double G = r2 + (1-e2)*z2 - e2*E2 ;
    double c = (e2*e2*F*r2)/(G*G*G) ;
    double s = pow(( 1 + c + sqrt(c*c + 2*c) ),1/3) ;
    double P = F / (3 * pow((s+1/s+1),2) * G*G) ;
    double Q = sqrt(1+2*e2*e2*P) ;
    double ro = -(P*e2*r)/(1+Q) + sqrt((a*a/2)*(1+1/Q) - (P*(1-e2)*z2)/(Q*(1+Q)) - P*r2/2) ;   
    double tmp = pow((r - e2*ro) , 2) ;
    double U = sqrt( tmp + z2 ) ;
    double V = sqrt( tmp + (1-e2)*z2 ) ;
    double zo = (b2*z)/(a*V) ;

    double height = U*( 1 - b2/(a*V) ) ;
    
    double lat = atan( (z + ep*ep*zo)/r ) ;

    double temp = atan(y/x) ;
    double lon;
    if (x >=0 )    
        lon = temp ;
    else 
        {if ((x < 0) & (y >= 0))
            lon = pi + temp ;
         else 
            lon = temp - pi ;
        }

    double lat0 = lat/(pi/180) ;
    double lon0 = lon/(pi/180) ;
    double h0 = height ;

    nav_msgs::Odometry msg_wgs = msg_ecef;
    msg_wgs.pose.pose.position.x = lat0;
    msg_wgs.pose.pose.position.y = lon0;
    msg_wgs.pose.pose.position.z = h0;

    return msg_wgs;
}


nav_msgs::Odometry enu_to_geodetic(nav_msgs::Odometry msg_enu, nav_msgs::Odometry msg_wgs0)
{
    return ecef_to_geodetic(enu_to_ecef(msg_enu, msg_wgs0));
}

void Send()
{
    int recv_num(0);

    if ((sockfd_1 = socket(AF_INET, SOCK_STREAM, 0)) == -1)
        {
            ROS_INFO("socket error!");
            return ;
        }
    bzero(&serv_addr_1, sizeof(serv_addr_1));
    serv_addr_1.sin_family = AF_INET;
    serv_addr_1.sin_port = htons(SERVPORT);
    serv_addr_1.sin_addr.s_addr = inet_addr(SERVER_IP_1);

    connect(sockfd_1, (struct sockaddr *) &serv_addr_1, sizeof(struct sockaddr)); 
    recv_num += write(sockfd_1, buffer+recv_num, strlen(buffer));
    cout <<"send:"<<recv_num << endl;

    close(sockfd_1);
}

class CPosPackage{
public:
    CPosPackage(ros::NodeHandle&);
    ~CPosPackage(){};
    //void registerPubSub();
    void ZCallback(const sensor_msgs::FluidPressure::ConstPtr& msg);
    void XYCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void Datapub();
private:
    double X,Y,Z;
    int floor_locaion=1;
    static const double H0;
    ros::Subscriber sub1;
    ros::Subscriber sub2;    
    ros::NodeHandle nh;
};


const double CPosPackage::H0=4.0;

int main(int argc, char **argv)
{

    ros::init(argc, argv, "Datapub");
    ros::NodeHandle nh; 
    ros::Rate s_timer(1); 
    // socket init  
    CPosPackage pub1(nh);

    sensor_msgs::FluidPressure msg_h;
    nav_msgs::Odometry msg_odometry;
    sensor_msgs::FluidPressure::ConstPtr msg_z_ptr(&msg_h);
    nav_msgs::Odometry::ConstPtr msg_xy_ptr(&msg_odometry);
    while(ros::ok())
    {       
        //ROS_INFO("I heard 83");
        s_timer.sleep(); // 在这里调用sleep函数可以让程序在这里
        ros::spinOnce();
    }
    return 0;
}

CPosPackage::CPosPackage(ros::NodeHandle& _nh):nh(_nh)
{   
    sub1 = nh.subscribe("/calPressure",1,&CPosPackage::ZCallback, this); //this pointer here means the class itselfn &tl1::fCallback
    sub2 = nh.subscribe("/vins_calib_slowrelease",1,&CPosPackage::XYCallback, this);
}

//对接受高度Z进行操作
void CPosPackage::ZCallback(const sensor_msgs::FluidPressure::ConstPtr& msg)
{
    Z = msg->fluid_pressure;
    //floor_locaion = int(ceil(Z/H0));
    //ROS_INFO("I heard 102 : [%d]",floor_locaion);
}

void CPosPackage::XYCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    nav_msgs::Odometry msg_init;
    //给起始坐标赋值
    msg_init.pose.pose.position.x = 39.964096551528826;
    msg_init.pose.pose.position.y = 116.35905684497429;
    msg_init.pose.pose.position.z = 39.83390;     
    nav_msgs::Odometry msg_;
    msg_ = *msg;
    nav_msgs::Odometry msg_gps;    
    msg_gps = enu_to_geodetic(msg_, msg_init);
    //cout << "main: " << setprecision(16) << msg_gps.pose.pose.position.x << endl;
    X = msg_gps.pose.pose.position.x;
    Y = msg_gps.pose.pose.position.y;
    Z = msg_gps.pose.pose.position.z;
    // X = 39.9640965288;
    // Y = 116.3590568450;  
    Datapub();
}

//对数据进行操作并发送
void CPosPackage::Datapub()
{    
    time_t now =time(0);
    struct tm *p;
    p=gmtime(&now);
    stringstream tim;
    tim <<1900+p->tm_year <<"/" 
         <<1+p->tm_mon  <<"/" 
         <<p->tm_mday <<"/" 
         << p->tm_hour+8 <<"/" 
         << p->tm_min <<"/" 
         << p->tm_sec;

    sprintf(buffer,"{\"Start\":true,\"Type\":\"@BUPTCOORD\",\"time\":\"%s\",\"DeviceNum\":2,\"Dimension\":3,\"X\":%.10f,\"Y\":%.10f,\"Z\":%.10f,\"floor\":%d,\"USERID\":3,\"End\":true}\n",
        tim.str().c_str(),(double)X, (double)Y, (double)Z, floor_locaion);  //time,x,y,z,floor  
     // socket transmit
    cout << buffer << endl;
    Send();
}

