//思路： 先转化为有序数组
// 再来查找众数,通过判断众数是否大于最低标准数S来判断楼层位置
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <sensor_msgs/FluidPressure.h>
#include <stdio.h>

#include <iostream>
#include <math.h>
using namespace std;

double Total_Height,Total_Floor;
#define min 8  //数组众数最小出现次数
#define N_filter 10 //数组长度
int floor_number;
double H;
bool base_flag=false;

void sortMethod(int intArray[],int array_size);
int zhongshuMethod(int intArray[],int array_size);

void location_floornumber(int result)
{
    
    if(result>= min)
    {cout << "floor number is :" << floor_number << endl;}
    else
    {cout << "Please waiting ..." << endl;}
}

void myCallback(const sensor_msgs::FluidPressure::ConstPtr& msg) 
{ 
  // 打印出接收到的值
  base_flag = true;
  //ROS_INFO("received drone_pressure is: %f",msg->fluid_pressure); 
  //ROS_INFO("received drone_temperature is: %f",msg->variance); 
  H=msg->fluid_pressure;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "location_floor");

    ros::NodeHandle nh;
    ros::Rate s_timer(10);
    std_msgs::Int8 msg;

    ros::Subscriber my_publisher_object1 = nh.subscribe("calPressure", 100, myCallback);
    ros::Publisher  my_publisher_object2 = nh.advertise<std_msgs::Int8>("location_floor", 100);
    
    if(nh.getParam("Total_Height", Total_Height))
        ROS_INFO("Total_Height is %f", Total_Height);
    else
        ROS_WARN("didn't find parameter Total_Height");
    
    if(nh.getParam("Total_Floor", Total_Floor))
        ROS_INFO("Total_Floor is %f", Total_Floor);
    else
        ROS_WARN("didn't find parameter Total_Floor");

    const int array_size = 10;
    int m=0,n=0;
    double buf[N_filter] = { 0 };
    int floor[10]={0};

    double H0=Total_Height/ Total_Floor;
    while (ros::ok()) 
    {
        if(base_flag)
        {
            base_flag = false;
            if(m<N_filter)
            {
                buf[m]=H;
                m++;
            }
            if(m==N_filter)
            {
                for (n=0;n<N_filter-1;n++)
                {
                  buf[n] = buf[n+1];
                }
                buf[N_filter-1] = H;
        
                for(int i=0; i<array_size; i++)
                {
                    floor[i] = int(ceil(buf[i] / H0));
                }
                sortMethod(floor,array_size);
                int result = zhongshuMethod(floor,array_size);
                location_floornumber(result);
                msg.data = floor_number;
                my_publisher_object2.publish(msg);
            }
        }
        s_timer.sleep(); // 在这里调用sleep函数可以让程序在这里
        ros::spinOnce();
    }
}





void sortMethod(int intArray[],int array_size)
{
    for(int i=0; i<array_size-1; i++)
    {
        for(int j=i+1; j<array_size; j++)
        {
            if(intArray[i] > intArray[j])
            {
                int temp;
                temp = intArray[i];
                intArray[i] = intArray[j];
                intArray[j] = temp;
            }
        }
    }
}
int zhongshuMethod(int intArray[],int array_size)
{
    int zhongshu;
    int currentFrequency = 0;
    int mostFrequency = 0;
    for(int i=0; i<array_size; i++)
    {
        currentFrequency++;
        if(intArray[i] != intArray[i+1] || i == array_size-1)
        {
            if(currentFrequency > mostFrequency)
            {
                mostFrequency = currentFrequency;
                zhongshu = intArray[i];
            }
            currentFrequency = 0;
        }
    }
    if(zhongshu<1)    
    {
        zhongshu=zhongshu-1;
    }
    floor_number = zhongshu;
    return mostFrequency;
}