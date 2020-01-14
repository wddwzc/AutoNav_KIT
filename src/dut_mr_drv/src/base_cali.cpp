#include "ros/ros.h"

#include <geometry_msgs/Twist.h>
#include "dut_mr_drv/dut_mr_msg.h"

#include "dut_mr_drv/RawMotorCmd.h"
#include "dut_mr_drv/MrHwInfo.h"

#include "pthread.h"
#include <boost/thread/thread.hpp> 

/** >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> **/
#define PI (3.1415926)
/** >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> **/
static boost::mutex mutex;

static dut_mr_drv::MrHwInfo base_hw_info;
static dut_mr_drv::MrHwInfo base_start_cali_hw_info;
static dut_mr_drv::MrHwInfo base_end_cali_hw_info;

static int start_sec;
static int end_sec;
static double WheelDistance,WheelDiameter,Wheelbase;
static int EncoderPulseNum;


static uint32_t left_adc_count = 0,right_adc_count = 0;
static uint64_t left_adc_sum = 0, right_adc_sum = 0;

static uint32_t start_left_adc_count = 0,start_right_adc_count = 0;
static uint64_t start_left_adc_sum = 0, start_right_adc_sum = 0;

static uint32_t tm_count = 0x00;


/** >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> **/
static void BaseHwInfo(const dut_mr_drv::MrHwInfo::ConstPtr &hw);

static void TxBaseCmdThread(ros::Publisher * arg);
/** >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> **/
int main(int argc,char**argv)
{
    ros::init(argc,argv,"dut_base_cali");
	
    ros::NodeHandle node;

	ros::Subscriber base_mr_hw_info_subsc = node.subscribe("/dut_base_link/hw_info",1000,BaseHwInfo);
	
	ros::Publisher *base_ctrl = new ros::Publisher;
	*base_ctrl =   node.advertise<dut_mr_drv::RawMotorCmd>("/dut_base_link/raw_motor_cmd",1000);


	boost::thread thrd(&TxBaseCmdThread,base_ctrl);

	ROS_INFO("ros spin now");
	
    ros::spin();
    return 0;
}

/** >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> **/
static void BaseHwInfo(const dut_mr_drv::MrHwInfo::ConstPtr &hw)
{
	mutex.lock();
		
	//base_hw_info = *hw;

	if ( tm_count < start_sec )
	{
		base_start_cali_hw_info = *hw;
		start_left_adc_count  += hw->fl_wheel_adc_count;
		start_right_adc_count += hw->fr_wheel_adc_count;
		start_left_adc_sum    += hw->fl_wheel_adc_val_sum;
		start_right_adc_sum   += hw->fr_wheel_adc_val_sum;
	}
	else if ( tm_count < end_sec )
	{
		left_adc_count 	+= hw->fl_wheel_adc_count;
		right_adc_count += hw->fr_wheel_adc_count;
		left_adc_sum    += hw->fl_wheel_adc_val_sum;
		right_adc_sum   += hw->fr_wheel_adc_val_sum;
	}
	else if ( tm_count == end_sec )
	{
		base_end_cali_hw_info = *hw;
		
		uint16_t aveg;
		double left_mV;
		
		aveg = (uint16_t)(left_adc_sum/left_adc_count);
		left_mV   = (double)left_adc_sum/left_adc_count * 3300/4096;
		ROS_INFO("Base Hw Info Left ADC Sum  Count=%d, Aveg = %d, Volt=%.2fmV",left_adc_count,aveg,left_mV);

		aveg = (uint16_t)(right_adc_sum/right_adc_count);
		double right_mV   = (double)right_adc_sum/right_adc_count * 3300/4096;
		ROS_INFO("Base Hw Info Right ADC Sum  Count=%d, Aveg = %d, Volt=%.2fmV",right_adc_count,aveg,right_mV);
		
		double start_left_mv = (double)start_left_adc_sum/start_left_adc_count * 3300/4096;
		double start_right_mv = (double)start_right_adc_sum/start_right_adc_count * 3300/4096;
		
		ROS_INFO("Base Cali Left Vollt: %.4fmV->%.4fmV(Chg:%.4fmV)",start_left_mv,left_mV,start_left_mv-left_mV);
		ROS_INFO("Base Cali Right Volt: %.4fmV->%.4fmV(Chg:%.4fmV)",start_right_mv,right_mV,start_right_mv-right_mV);
				
		int left_while_dis  = base_end_cali_hw_info.rl_wheel_encode_val - base_start_cali_hw_info.rl_wheel_encode_val;
		int right_while_dis = base_end_cali_hw_info.rr_wheel_encode_val - base_start_cali_hw_info.rr_wheel_encode_val;

		ROS_INFO("Left Wheel Encode:  %d -> %d, Dis = %d",base_start_cali_hw_info.rl_wheel_encode_val,base_end_cali_hw_info.rl_wheel_encode_val,left_while_dis);
		ROS_INFO("Right Wheel Encode: %d -> %d, Dis = %d",base_start_cali_hw_info.rr_wheel_encode_val,base_end_cali_hw_info.rr_wheel_encode_val,right_while_dis);

		double left_dis_m  = (double)left_while_dis /EncoderPulseNum*WheelDiameter*PI;
		double right_dis_m = (double)right_while_dis/EncoderPulseNum*WheelDiameter*PI;
		ROS_INFO("Base Forward Wheel: Left = %.4fm, Right = %.4fm",left_dis_m,right_dis_m);


		if ( left_while_dis != 0 && right_while_dis != 0 && left_while_dis != right_while_dis)
		{
			/* calc R first */
			if ( left_dis_m < right_dis_m )
			{	// turn  left
				double angle 		= 	(right_dis_m-left_dis_m)*2*PI/WheelDistance;
				double turn_radius	=	left_dis_m*2*PI/angle + WheelDistance/2;

				ROS_INFO("Left Turn Angle = %.4f, Radius = %.4f, Sensor Volt Left = %.4fmV, Right = %.4f",angle,turn_radius,left_mV,right_mV);		
			
				double steer_angle = atan(Wheelbase/turn_radius);
				//ROS_INFO("Turn Steer:%f",steer_angle);
				ROS_INFO("Turn Steer:%f rad, (%f) ",steer_angle,steer_angle/PI*180);
			}
			else
			{	// Turn Right
			
				double angle 		= 	(left_dis_m - right_dis_m)*2*PI/WheelDistance;
				double turn_radius	=	right_dis_m*2*PI/angle + WheelDistance/2;

				ROS_INFO("Right Turn Angle = %.4f, Radius = %.4f, Sensor Volt Left = %.4fmV, Right = %.4f",angle,turn_radius,left_mV,right_mV);			
				double steer_angle = atan(Wheelbase/turn_radius);
				ROS_INFO("Turn Steer:%f rad, (%f) ",steer_angle,steer_angle/PI*180);
			}
		}
		else if ( left_while_dis == right_while_dis )
		{
			ROS_ERROR("Base Cali Error left_while_dis == right_while_dis");
		}
		else
		{
			ROS_ERROR("Base Cali Error !!!!!!!...");
		}
		
		ROS_INFO("Base Hw Info Cali Complete");
	}
	else{}
	
	mutex.unlock();

	//ROS_INFO("Base Hw Info.. Tick =%d",base_hw_info.mr_tick);

	
}

/** >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> **/
static void TxBaseCmdThread(ros::Publisher *raw_ctrl_pub)
{
	dut_mr_drv::RawMotorCmd raw_motor_cmd;

    ros::NodeHandle node("~");
	
	int32_t ctrl_method;
	int32_t steering_motor_pos;
	int drv_motor_pwm_duty;

	
    node.param(std::string("BaseMethodCtrl"),ctrl_method,0);
	node.param(std::string("SteerMotorPos"),steering_motor_pos,0);
	node.param(std::string("DrvMotorPwm"),drv_motor_pwm_duty,0);
	
	node.param(std::string("CaliStart"),start_sec,5);
	node.param(std::string("CaliEnd"),end_sec,10);
	
	node.param(std::string("WheelDistance"),WheelDistance,0.30);
	node.param(std::string("WheelDiameter"),WheelDiameter,0.30);
	node.param(std::string("Wheelbase"),Wheelbase,0.85);
	node.param(std::string("EncoderPulseNum"),EncoderPulseNum,1000);

	

	ROS_INFO("Base Wheel info:WheelDistance=%.3f,WheelDiameter=%.3f,Wheelbase=%.3f,EncoderPulseNum=%d",WheelDistance,WheelDiameter,Wheelbase,EncoderPulseNum);
		

	raw_motor_cmd.ctrl_method 			= ctrl_method;
	raw_motor_cmd.steering_motor_pos 	= steering_motor_pos;
	raw_motor_cmd.drv_motor_pwm_duty 	= drv_motor_pwm_duty;

	ROS_INFO("ROS Cali Cmd: Ctrl_method=0x%04x, steer pos=%d,pwm_duty=%d",raw_motor_cmd.ctrl_method,raw_motor_cmd.steering_motor_pos,raw_motor_cmd.drv_motor_pwm_duty);
	ROS_INFO("Cali Start Sec=%d, End Sec=%d",start_sec,end_sec);

	ROS_INFO("TxBaseCmdThread Start Ok");
	
	ros::Rate loop(10);
	
	/* initial base control command*/
	start_sec = start_sec*10;
	end_sec   = end_sec * 10;


	
	while( ros::ok() )
	{
		loop.sleep();

		tm_count ++ ;

		if ( tm_count < start_sec )	// wait 10s
		{
			raw_motor_cmd.drv_motor_pwm_duty = 0;
		}
		else if ( tm_count < end_sec )
		{
			raw_motor_cmd.drv_motor_pwm_duty = drv_motor_pwm_duty;
		}
		else if ( tm_count == end_sec )
		{	// output cali result
			raw_motor_cmd.steering_motor_pos = 0;
		}
		else
		{
			raw_motor_cmd.drv_motor_pwm_duty = 0;
		}
		raw_ctrl_pub->publish(raw_motor_cmd);
	}
}
/** >>>>>>>>>>>>>>>>>>>>>>>>>>> end of file >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> **/



