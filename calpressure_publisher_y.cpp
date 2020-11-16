#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/FluidPressure.h>
#include <stdio.h>
#include <math.h>
//滤波长度
#define N_filter 10

double H0,P0,P,Pm;
bool base_flag1=false;
bool base_flag2=false;
std_msgs::Header p_head;
 //基站气压、温度
void baseCallback(const sensor_msgs::FluidPressure::ConstPtr&  msg) 
{ 
  // 打印出接收到的值
  base_flag1 = true;
//  ROS_INFO("received base_pressure is: %f",msg->fluid_pressure); 
  //ROS_INFO("received base_temperature is: %f",msg->variance); 
  P0=msg->fluid_pressure;

}
//无人机气压
void myCallback(const sensor_msgs::FluidPressure::ConstPtr& msg) 
{ 
  // 打印出接收到的值
  base_flag2 = true;
//  ROS_INFO("received drone_pressure is: %f",msg->fluid_pressure); 
  //ROS_INFO("received drone_temperature is: %f",msg->variance); 
  P=msg->fluid_pressure/100.0;
  p_head=msg->header;
}

//中值滤波
double medFilter(double * DataBuffer)
{
  int i, j;// 循环变量
  double temp;
   // 用冒泡法对数组进行排序
  for (j = 0; j < N_filter - 1; j++)
   {
      for (i = 0; i < N_filter - j - 1; i++)
      {
        if (DataBuffer[i] > DataBuffer[i + 1])
         {
            temp = DataBuffer[i];
            DataBuffer[i] = DataBuffer[i + 1];
            DataBuffer[i + 1] = temp;
         }
      }
  }
   // 计算中值
  if ((N_filter & 1) > 0)
  {
     // 数组有奇数个元素，返回中间一个元素
     temp = DataBuffer[(N_filter-1)/2];
      //printf("%d", temp);
  }
  else
  {
      // 数组有偶数个元素，返回中间两个元素平均值
      temp = (DataBuffer[N_filter/2-1] + DataBuffer[N_filter/2])/2.0;
     //printf("%d\n", DataBuffer[N_filter / 2]);
      //printf("%d\n", DataBuffer[N_filter / 2]+1);
  }
  return temp;
}


int main(int argc, char **argv) 
{    
    const double Rd=287,g=9.8; 
    int count=0;
    double ret;
   //存放数据buf
    double Pb[N_filter]={0};
    double buf[N_filter] = { 0 };
    double buf_copy[N_filter] = { 0 };
    int m=0, n=0,k=0;

    ros::init(argc, argv, "calbarometer_publisher"); // 初始化节点名
    ros::NodeHandle nh; // 
    std_msgs::Float64 H;
    sensor_msgs::FluidPressure H_; 
    ros::Rate s_timer(10); // 参数1.0代表发布频率即1.0Hz
    ros::Publisher  my_publisher_object = nh.advertise<sensor_msgs::FluidPressure>("calPressure", 100); 
    ros::Subscriber my_subscriber_object1= nh.subscribe("pressure",110,baseCallback); 
    ros::Subscriber my_subscriber_object2= nh.subscribe("/mavros/imu/static_pressure",110,myCallback); 

    if(nh.getParam("H0", H0))
        ROS_INFO("H0 is %f", H0);
    else
        ROS_WARN("didn't find parameter H0");
    if(nh.getParam("Pm", Pm))
        ROS_INFO("Pm is %f", Pm);
    else
        ROS_WARN("didn't find parameter Pm");
    // 程序所要做的工作将在下面的循环里完成
    while (ros::ok()) 
    {
//        if(count<10&&base_flag1&&base_flag2)
//         {
//           base_flag1 = false;
//           base_flag2 = false;
//           Pb[count]=P0-P;
// //          ROS_INFO("pressure bias is: %f",Pb/count);//想要除以i
//           count++;
//         }

       if(base_flag1&&base_flag2)
       {
          base_flag1 = false;
          base_flag2 = false;
//          Pm=medFilter(Pb);
          H.data=H0+44330*(1-pow(P/(P0+Pm),(1.0/5.225)));

          if(m<N_filter)
          {
            buf[m]=H.data;
            m++;
          }
          if(m==N_filter)
          {
            for (n=0;n<N_filter-1;n++)
            {
              buf[n] = buf[n+1];
            }
            buf[N_filter-1] = H.data;

            memcpy(buf_copy,buf,N_filter*sizeof(buf_copy[0]));
            ret = medFilter(buf_copy);
            ROS_INFO("Ret Height is :%f", ret);
          }
          H_.fluid_pressure=H.data;

          H_.header=p_head;
          my_publisher_object.publish(H_); // 发布消息到对应的话题
       }
       s_timer.sleep(); // 在这里调用sleep函数可以让程序在这里
       ros::spinOnce();
    }
}




