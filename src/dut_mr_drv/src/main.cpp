
#include "ros/ros.h"

#include <geometry_msgs/Twist.h>
#include "dut_mr_drv/dut_mr_msg.h"
#include "dut_mr_drv/RawMotorCmd.h"

static DutMrMsg *dut_mr_msg;

void CmdVelSubCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
	ROS_INFO("req speed;linear_x=%f;angular_z=%f",msg->linear.x,msg->angular.z);
	dut_mr_msg->SetMrSpeed(msg->linear.x, msg->angular.z);
}

void CmdRamMotorCmd(const dut_mr_drv::RawMotorCmd::ConstPtr &cmd)
{
	dut_mr_msg->SetMrRawMotorCmd(cmd);
	ROS_DEBUG("req raw motor ctrl;steer motor pos=%d;drv motor duty=%d",cmd->steering_motor_pos, cmd->drv_motor_pwm_duty);
}

int main(int argc,char**argv)
{
    ros::init(argc,argv,"dut_mr");
	
    ros::NodeHandle node;

	dut_mr_msg = new DutMrMsg;

	//dut_mr_msg->SetMrStop();
	
	ros::Subscriber cmd_subsc 			= node.subscribe("cmd_vel",1000,CmdVelSubCallback);
	ros::Subscriber raw_motor_cmd_subsc = node.subscribe("raw_motor_cmd",1000,CmdRamMotorCmd);

	ROS_INFO("ros spin now");
  //  Model d_robot;
  //  if(!d_robot.Init())return -1;
  //  if(d_robot.cmdMotors(true))return -1;
   // d_robot.startSample();
    ros::spin();
    return 0;
}



