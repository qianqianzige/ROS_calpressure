#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/FluidPressure.h>


double H0,P0,P,T_base,T_drone;
bool base_flag=false;
//基站气压、温度
void baseCallback(const sensor_msgs::FluidPressure::ConstPtr&  msg1) 
{ 
  // 打印出接收到的值
  ROS_INFO("received pressure is: %f",msg1->fluid_pressure); 
  ROS_INFO("received temperature is: %f",msg1->variance); 
  P0=msg1->fluid_pressure;
  T_base=msg1->variance;
}
//无人机气压
void myCallback(const sensor_msgs::FluidPressure::ConstPtr& msg2) 
{ 
  // 打印出接收到的值
  base_flag = true;
  ROS_INFO("received value is: %f",msg2->fluid_pressure); 
  P=msg2->fluid_pressure;
  T_drone=msg2->variance;
}


int main(int argc, char **argv) 
{    
    const double  Rd=287,g=9.8; 
    double Tm;
    ros::init(argc, argv, "calbarometer_publisher"); // 初始化节点名
    ros::NodeHandle n; // 
    std_msgs::Float64 H;
    ros::Rate s_timer(10); // 参数1.0代表发布频率即1.0Hz
    ros::Publisher  my_publisher_object = n.advertise<std_msgs::Float64>("calPressure", 100); 
    ros::Subscriber my_subscriber_object1= n.subscribe("bar_base",110,baseCallback); 
    ros::Subscriber my_subscriber_object2= n.subscribe("bar_drone",110,myCallback); 
    // 程序所要做的工作将在下面的循环里完成
    while (ros::ok()) 
    {
       if(base_flag)
       {
         base_flag = false;
         Tm = (T_drone+T_base)/2.0;
         H.data=H0+log(10)*Rd/g*(273.15+Tm)*(log10(P0)-log10(P));
         my_publisher_object.publish(H); // 发布消息到对应的话题
       }
        s_timer.sleep(); // 在这里调用sleep函数可以让程序在这里
        ros::spinOnce();

    }
}
