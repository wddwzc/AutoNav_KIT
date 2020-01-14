#include "ros/ros.h"
#include <iostream>
#include <fstream>

//#include <geometry_msgs/Twist.h>
//#include "dut_mr_drv/RawMotorCmd.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include "gnss_driver/gpgga_msg.h"
#include "gnss_driver/headinga_msg.h"

/** >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> **/

//#ifndef __pi__
//#define __pi__
const double PI = 3.1415926535898;
//#endif

double yaw = 0.0;
double x_start = 0.0;
double y_start = 0.0;
double yaw_start = 0.0;
bool xy_first = true;
bool yaw_first = true;

ros::Publisher pub_odom;

std::ofstream ouf("/home/robot/topology_map.txt");

/** >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> **/


void GPStoXY(double longtitude,double latitude,double &x,double &y)
{
	const double L0 = 121.5215999;
	const double C = 6399596.65198801; // 
	const double ee = 0.00673951819500;

	const double PI = 3.1415926535;

	double sinB = sin(PI/180*latitude);
	double cosB = cos(PI/180*latitude);
	double X = 111134.0047*latitude-(32009.8575*sinB+133.9602*sinB*sinB*sinB
				+0.6976*sinB*sinB*sinB*sinB*sinB+0.0039*sinB*sinB*sinB*sinB*sinB*sinB*sinB)*cosB;
	double t = tan(PI/180*latitude);
	double l = longtitude - L0;
	double m = PI/180*l*cosB;
	double nn = ee*cosB*cosB;
	double N = C/sqrt(1+nn);
	x = X + N*t*(0.5*m*m)+(double)1/24*(5-t*t+4*nn*nn)*m*m*m*m
			+(double)1/720*(61-58*t*t+t*t*t*t*m*m*m*m*m*m);
	y = N*(m+(double)1/6*(1-t*t+nn)*m*m*m+(double)1/120*(5-18*t*t+t*t*t*t+14*nn-58*nn*t*t)*m*m*m*m*m);
}

/** >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> **/

//	void CmdVelSubCallback(const geometry_msgs::Twist::ConstPtr &msg)
//	{
//		ROS_INFO("req speed;linear_x=%f;angular_z=%f",msg->linear.x,msg->angular.z);
//	}
//	
//	void CmdRamMotorCmd(const dut_mr_drv::RawMotorCmd::ConstPtr &cmd)
//	{
//		ROS_DEBUG("req raw motor ctrl;steer motor pos=%d;drv motor duty=%d",cmd->steering_motor_pos, cmd->drv_motor_pwm_duty);
//	}

void GnssDrvPos(const gnss_driver::gpgga_msg::ConstPtr &pos)
{
	//ROS_DEBUG("req raw motor ctrl;steer motor pos=%d;drv motor duty=%d",cmd->steering_motor_pos, cmd->drv_motor_pwm_duty);
	double lat = pos->latitude;
	double lon = pos->longitude;

	double x,y;

	GPStoXY(pos->longitude,pos->latitude,x,y);

	y = -y;

	if (xy_first) {
		x_start = x;
		y_start = y;
		xy_first = false;
	}
	x -= x_start;
	y -= y_start;

	// std::cout << "#### Pose" << std::endl;
	// std::cout << "GPS Coordinate Lat = " << pos->latitude << ", Lon = " << pos->longitude << std::endl;
	std::cout << "Start: X = " << x_start << ", Y = " << y_start << std::endl;
	std::cout << "Base coordinate: X = " << x << ", Y = " << y << std::endl;
	nav_msgs::Odometry poseData;
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(yaw);

	poseData.header.frame_id = "/camera_init";

	poseData.pose.pose.position.x = x;
	poseData.pose.pose.position.y = y;
	poseData.pose.pose.position.z = 0.0;
	poseData.pose.pose.orientation = odom_quat;

	ouf << 0 << " " << 0 << " " << 0 << " "
		<< x << " " << y << " " << 0.0 << " "
		<< yaw << " " << 0.0 << " " << 0.0 << " "
		<< 0.0 << " " << 0.0 << " " << 0.0 << " "
		<< 0.0 << " " << 0.0 << " " << 0.0 << " "
		<< 0.0 << " " << 0.0 << " " << 0 << " "
		<< 0 << " " << 0 << " " << 0 << " "
		<< 0.0 << " " << 0.0 << " " << 0.0 << std::endl;

	pub_odom.publish(poseData);
}

void HeadingInfo(const gnss_driver::headinga_msg::ConstPtr &heading)
{
	if (yaw_first) {
		yaw_start = heading->yawAngle;
		yaw_first = false;
	}
	if (heading->calculateStatus == ";SOL_COMPUTED" && heading->locateStatus != "NONE") {
		yaw = 360 - heading->yawAngle;
		std::cout << "#### Angle   " << yaw << std::endl;
	}
	else {
		std::cout << "#### Angle   " << yaw << "   NONE" << std::endl;
	}
	
	
	// std::cout << "#### Angle" << std::endl;
	// std::cout << "start = " << yaw_start << "     yawAngle = " << yaw << std::endl;

	// yaw -= yaw_start;

	yaw = yaw / 180 * PI;


	// if (yaw > PI) {
	// 	yaw -= PI * 2;
	// }
	// if (yaw < -PI) {
	// 	yaw += PI * 2;
	// }
}

int main(int argc,char**argv)
{
    ros::init(argc,argv,"dut_base_path_planner");
	
    ros::NodeHandle node;
	
//	ros::Subscriber cmd_subsc 			= node.subscribe("cmd_vel",1000,CmdVelSubCallback);
//	ros::Subscriber raw_motor_cmd_subsc = node.subscribe("raw_motor_cmd",1000,CmdRamMotorCmd);

    pub_odom = node.advertise<nav_msgs::Odometry>("/chatter", 100);

	ros::Subscriber gnss_gpgga 	= node.subscribe("/gnss/data",1000,GnssDrvPos);
	ros::Subscriber heading	 	= node.subscribe("/heading/angle",1000,HeadingInfo);

	ROS_INFO("ros spin now");
	
	//  Model d_robot;
	//  if(!d_robot.Init())return -1; 
	//  if(d_robot.cmdMotors(true))return -1;
	// d_robot.startSample();
	
    ros::spin();

    ouf.close();

    return 0;
}
