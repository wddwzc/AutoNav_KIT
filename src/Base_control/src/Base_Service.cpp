#include <ros/ros.h>
#include <iostream>
#include <string>
#include "Base_control/ControlCmd.h"
#include <geometry_msgs/Twist.h>

double linear_coeff, angular_coeff;

ros::Publisher pub_cmd;
//-----------------回调函数---------------------------
//*********包名::服务名::请求/服务 类定义*************
//---------------------------------------------------
bool handlefunction(Base_control::ControlCmd::Request &req, Base_control::ControlCmd::Response &res)
{
	std::cout<<"Base_control recv......" << " linear_velocity  "<< req.xx <<"  angular_velocity  "<< req.yy << std::endl;
  float Vlinear = (double) req.xx * linear_coeff / 1000;
  float Vangular = (double) req.yy * angular_coeff / 1000;
  res.zz = Vlinear * 10 + Vangular;
  geometry_msgs::Twist speed;
  speed.linear.x = Vlinear;
  speed.angular.z = Vangular;
  pub_cmd.publish(speed);
  return true;
}

//-----------------服务主题函数--------------------------- 
int main(int argc, char **argv)
{
   ros::init(argc, argv, "app_Controlservice");
   ros::NodeHandle n;
   ros::NodeHandle np("~");
   np.param<double>(std::string("linear_factor"), linear_coeff, 1.0);
   np.param<double>(std::string("angular_factor"), angular_coeff, 1.0);
   ros::ServiceServer service = n.advertiseService("Controler", handlefunction);
   pub_cmd = n.advertise<geometry_msgs::Twist>("/base_move", 10);
   // ros::Rate loop_rate(50);
   ROS_INFO("Controler service start!!!");
   while (ros::ok()) {
    ros::spin();
   }
   return 0;
}
