
#include"ros/ros.h"
#include "dut_mr_drv/dut_mr_msg.h"
#include "thread"
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include "pthread.h"

#include <boost/thread/thread.hpp> 

#include "dut_mr_drv/MrHwInfo.h"

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>


/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> */
/*
*/

/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> */
using std::string;

/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> */
#define				UART_BYTE_START		0x81
#define				UART_BYTE_END		0x82
#define				UART_BYTE_ESCAPE	0x80

/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> */

/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> */

/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> */
DutMrMsg::DutMrMsg():heartbeat_tick{0}
{
    ros::NodeHandle node("~");

	std::string SerialPortName;
	int	 		baudrate;
	
    node.param(std::string("SerialPortName"),SerialPortName,std::string("/dev/ttyUSB0"));
	node.param(std::string("SerialBaudrate"),baudrate,115200);
	
	rs232 =new RS232;
	rs232->m_configSerial(SerialPortName, baudrate,rs232->m_Ser );


	odom_pub = node.advertise<nav_msgs::Odometry>("Odom",50);

	boost::function0<void> func_tx = boost::bind(&DutMrMsg::TxUartMsgThread, this);
	boost::thread thrd_tx(func_tx);
	
	boost::function0<void> func_rx = boost::bind(&DutMrMsg::RxUartMsgThread, this);
	boost::thread thrd_rx(func_rx);
}

/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> */
//	void DutMrMsg::SetMrStop()
//	{
//		ROS_INFO("DUT MR Require Stop ...");
//	}

/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> */
void DutMrMsg::SetMrSpeed(double linear_x, double angular_z)
{
	mutex_cmd.lock();
	
	//ROS_INFO("DUT MR Require Speed x=%f, z=%f",linear_x,angular_z);
	req_linear_x  = linear_x;
	req_angular_z = angular_z; 

	req_twist_msg = true;	
	mr_twist_ctrl = true;

	mutex_cmd.unlock();	
}

void DutMrMsg::SetMrRawMotorCmd(const dut_mr_drv::RawMotorCmd::ConstPtr &cmd)
{
	mutex_cmd.lock();
	raw_motor_cmd = *cmd;
		
//		raw_motor_cmd.ctrl_method 			= cmd->ctrl_method;
//		raw_motor_cmd.steering_motor_pos 	= cmd->steering_motor_pos;
//		raw_motor_cmd.left_steer_angle 		= cmd->left_steer_angle;
//		raw_motor_cmd.right_steer_angle 	= cmd->right_steer_angle;
//		raw_motor_cmd.left_steer_adc_val 	= cmd->left_steer_adc_val;
//		raw_motor_cmd.right_steer_adc_val 	= cmd->right_steer_adc_val;
//		raw_motor_cmd.drv_motor_pwm_duty 	= cmd->drv_motor_pwm_duty;

	req_twist_msg 	= true;
	mr_twist_ctrl 	= false;

	mutex_cmd.unlock();
	
}
/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> */
void DutMrMsg::GetMrPosition(double & x, double & y, double & theta, double & linear_x, double & angular_z)
{
	x 			= mr_postion.x;
	y 			= mr_postion.y;
	theta 		= mr_postion.theta;
	linear_x 	= mr_postion.linear_x;
	angular_z 	= mr_postion.angular_z;

	ROS_DEBUG("GetMrPosition,x=%f,y=%f,theta=%f,linear_x=%f,angular_z=%f",x,y,theta,linear_x,angular_z);
}
/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> */
void DutMrMsg::UartRxMsgInfoOut(void)
{
	string str;
	
	for ( size_t i = 0; i < uart_rx_len; i++ )
	{
		char asc[4];
		sprintf(asc, "0x%02x ", uart_rx_buf[i]);
	
		str += asc;
	}
	ROS_INFO("rx byte len = %d; %s",(int)uart_rx_len,str.c_str());
}
/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> */
void DutMrMsg::UartDecodeRxMsgInfoOut(void)
{
	string str;
	
	for ( size_t i = 0; i < uart_msg_len; i++ )
	{
		char asc[4];
		sprintf(asc, "0x%02x ", uart_msg_cmd_buf[i]);
	
		str += asc;
	}
	ROS_DEBUG("uart message = %d; %s",(int)uart_msg_len,str.c_str());	
}
/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> */
void DutMrMsg::UartRxMsgParse(void)
{
	for ( auto i = 0; i < uart_rx_len; i++ )
	{
		uint8_t &rcv = uart_rx_buf[i];

		uart_msg_crc += rcv;
		switch(uart_msg_rcv_state)
		{
		case UART_MSG_RECV_NORMAL:
		    switch(rcv)
	        {
	        case UART_BYTE_START:	// re-start message
	            uart_msg_len  		= 0;
	            uart_msg_crc   		= UART_BYTE_START;
	            uart_msg_rcv_state 	= UART_MSG_RECV_NORMAL;
				ROS_ERROR("Received Uart Message error: rx re-start message...");
	            break;

	        case UART_BYTE_END:	//receive complete
	            /* receive message handle */
	            if ( uart_msg_crc == 0x00 )
	            {
	            	if ( uart_msg_len > 2 )
	            	{
	            		uart_msg_len -= 1;//delete crc count
	            		UartDecodeRxMsgInfoOut();
						for ( uint16_t i = 0; i < sizeof(uart_cmd_proc_tbl)/sizeof(UART_CMD_PROCESS); i ++ )
						{
							UART_CMD_PROCESS const *func = uart_cmd_proc_tbl + i;
							if ( func->process != NULL && func->msg_key[0] == uart_msg_cmd_buf[0] && func->msg_key[1] == uart_msg_cmd_buf[1])
							{
								func->process(this,uart_msg_cmd_buf+2,uart_msg_len-2);
								break;
							}
						}
	            	}
	            	else
	            	{// msg too short
	            		ROS_ERROR("Received Uart Message error: too short");
	            	}
	            }
	            else
	            {//msg crc error
					ROS_ERROR("Received UART Message, CRC error..");
	            }
	            uart_msg_rcv_state = UART_MSG_WAIT_START;
	            break;

	        case UART_BYTE_ESCAPE:
	            uart_msg_rcv_state = UART_MSG_RECV_ESCAPE;
	            break;

	        default:
	            if ( uart_msg_len < sizeof(uart_msg_cmd_buf) )
	            {
	               	uart_msg_cmd_buf[uart_msg_len++] = 	rcv;
	            }
	            else
	            {
	                uart_msg_rcv_state = UART_MSG_WAIT_START;
	            }
	            break;
	        }
			break;

		case UART_MSG_RECV_ESCAPE:
	        if (( uart_msg_len < sizeof(uart_msg_cmd_buf)) && (rcv < 4) )
	        {
	            uart_msg_cmd_buf[uart_msg_len++] = 	rcv+UART_BYTE_ESCAPE;
	            uart_msg_rcv_state			 	= 	UART_MSG_RECV_NORMAL;
	        }
	        else
	        {
	            uart_msg_rcv_state = UART_MSG_WAIT_START;
	            ROS_ERROR("Received UART Escape Code last, this code need 0,1,2 or 3,rcv = 0x%02x,msg=0x%02x%02x%02x",rcv,uart_msg_cmd_buf[0],uart_msg_cmd_buf[1],uart_msg_cmd_buf[2]);
	        }
			break;

		case UART_MSG_WAIT_START:
		default:
		    if ( rcv == UART_BYTE_START )
	        {
	            uart_msg_len  		= 0;
	            uart_msg_crc   		= UART_BYTE_START;
	            uart_msg_rcv_state 	= UART_MSG_RECV_NORMAL;
	        }
			else
			{
				ROS_ERROR("Received UART Message Error, need start byte message..");
			}
			break;
		}		
	}

}


/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> */
void DutMrMsg::UartTxMsgCmd(uint8_t code,uint8_t subcode,uint8_t *cmd,uint8_t cmd_len)
{
	size_t len = 0x00;
	uint16_t i;
	uint8_t  cs;
	
	uint8_t msg_cmd[512];
	msg_cmd[len++] = 0x81;	// start of frame	
	//msg_cmd[len++] = 0x01;
	msg_cmd[len++] = code;
	msg_cmd[len++] = subcode;

	cs = (msg_cmd[0] + msg_cmd[1] + msg_cmd[2] /*+ msg_cmd[3]*/);


	for ( i = 0x00; i < cmd_len; i ++ )
	{
		if ( cmd[i] == 0x80 || cmd[i] == 0x81 || cmd[i] == 0x82 )
		{
			msg_cmd[len++] = 0x80;
			msg_cmd[len++] = cmd[i] - 0x80;
		}
		else
		{
			msg_cmd[len++] = cmd[i];
		}		
		cs += cmd[i];
	}


	cs += 0x82;	// add end of frame
	cs = (uint8_t)(0x100 - cs);
	if ( cs == 0x80 || cs == 0x81 || cs == 0x82 )
	{
		msg_cmd[len++] = 0x80;
		msg_cmd[len++] = cs - 0x80;
	}
	else
	{
		msg_cmd[len++] = cs;
	}
	msg_cmd[len++] = 0x82;	// add end of frame

	rs232->m_Ser->write(msg_cmd, len);
}


/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> */
void DutMrMsg::RxUartMsgThread(void)
{
	ROS_INFO("RxMsgThreadHandle start..");
	ros::NodeHandle node("~");
	hw_info_pub =  node.advertise<dut_mr_drv::MrHwInfo>("hw_info",1000);
	
	while(ros::ok())
	{
		uart_rx_len  = rs232->m_Ser->read(uart_rx_buf, 1024);
		if ( uart_rx_len > 0 )
		{
			UartRxMsgParse();		// parse uart message raw data
		}
	}
}
/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> */


/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> */
void DutMrMsg::TxUartMsgThread(void)
{
	ROS_INFO("TxMsgThreadHandle start..");

	ros::Rate loop(5);
	uint8_t cmd = 0x00;					// for ROS RAW motor control
	//uint8_t cmd = 0x01;				// for ROS x,z command
	UartTxMsgCmd(0x02,0x01,&cmd,1);  	// TX command	
	
	while(ros::ok())
	{
		heartbeat_tick++;
		
		uint8_t msg_raw[5];
		uint8_t *p = (uint8_t*)&heartbeat_tick;
		msg_raw[0] = p[0];
		msg_raw[1] = p[1];
		msg_raw[2] = p[2];
		msg_raw[3] = p[3];
		msg_raw[4] = (uint8_t)mr_twist_ctrl;
		UartTxMsgCmd(0xEF,0x01,msg_raw,5);	// for tick test .code=0xFF

		mutex_cmd.lock();
			
		if ( mr_twist_ctrl )
		{
			/* tx req_linear_x,req_angular_z to controller */
			float x,z;
			if ( req_twist_msg )
			{
				req_twist_msg = false;
				
				x = (float)req_linear_x;
				z = (float)req_angular_z;
			}
			else
			{
				x = 0.0;
				z = 0.0;
			}

			ROS_DEBUG("MR cmd linear_x= %f, Angular_z = %f",x,z);

			uint8_t cmd[8];

			uint8_t *p;

			p 		= 	(uint8_t*)(&x);
			cmd[0]	=	p[0];
			cmd[1]	=	p[1];
			cmd[2]	=	p[2];
			cmd[3]	=	p[3];
			p 		= 	(uint8_t*)(&z);
			cmd[4]	=	p[0];
			cmd[5]	=	p[1];
			cmd[6]	=	p[2];
			cmd[7]	=	p[3];

			mutex_cmd.unlock();

			UartTxMsgCmd(0x90,0x01,cmd,8);	//code = 0x90,subcode = 0x01			
		}
		else
		{
			
			uint32_t 	ctrl_method;
	
			int32_t pos;
			int16_t pwm;
			
			if ( req_twist_msg )
			{
				req_twist_msg 	= false;
				ctrl_method 	= raw_motor_cmd.ctrl_method;
			}
			else
			{
				ctrl_method = 0x00;

			}

			uint8_t 	cmd[256];
			uint16_t  	len 	= 0;
			uint8_t 	*p;
			
			p 			= 	(uint8_t*)(&ctrl_method);
			cmd[len++]	=	*p++;
			cmd[len++]	=	*p++;
			cmd[len++]	=	*p++;
			cmd[len++]	=	*p++;
			p 			= 	(uint8_t*)(&raw_motor_cmd.steering_motor_pos);
			cmd[len++]	=	*p++;
			cmd[len++]	=	*p++;
			cmd[len++]	=	*p++;
			cmd[len++]	=	*p++;
			p 			= 	(uint8_t*)(&raw_motor_cmd.left_steer_angle);
			cmd[len++]	=	*p++;
			cmd[len++]	=	*p++;
			cmd[len++]	=	*p++;
			cmd[len++]	=	*p++;
			p 			= 	(uint8_t*)(&raw_motor_cmd.right_steer_angle);
			cmd[len++]	=	*p++;
			cmd[len++]	=	*p++;
			cmd[len++]	=	*p++;
			cmd[len++]	=	*p++;
			p 			= 	(uint8_t*)(&raw_motor_cmd.left_steer_adc_val);
			cmd[len++]	=	*p++;
			cmd[len++]	=	*p++;
			p 			= 	(uint8_t*)(&raw_motor_cmd.right_steer_adc_val);
			cmd[len++]	=	*p++;
			cmd[len++]	=	*p++;
			p 			= 	(uint8_t*)(&raw_motor_cmd.drv_motor_pwm_duty);
			cmd[len++]	=	*p++;
			cmd[len++]	=	*p++;
			
			mutex_cmd.unlock();

			
			UartTxMsgCmd(0x90,0x02,cmd,len);	//code = 0x90,subcode = 0x02		
		}

		loop.sleep();
	}
}



/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> */
void DutMrMsg::McuTickMsgRx(uint8_t const *msg, uint16_t const len)
{	
	//ROS_DEBUG("MCU tick rx, cnt,%d, %d, %d, %d",msg[0],msg[1],msg[2],msg[3]);
}

void DutMrMsg::ProcRxMsgMrHwInfo(uint8_t const *msg, uint16_t const len)	//02 01
{	//tick,drv_motor_pos(4),pwm_duty,mot_current,freq,ctrl_compl

	dut_mr_drv::MrHwInfo hw_info;

	MsgConvToData(&hw_info.mr_tick,msg);
	msg += sizeof(hw_info.mr_tick);
	
	MsgConvToData(&hw_info.drv_mot.pos,msg);
	msg += sizeof(hw_info.drv_mot.pos);
	MsgConvToData(&hw_info.drv_mot.duty,msg);
	msg += sizeof(hw_info.drv_mot.duty);
	MsgConvToData(&hw_info.drv_mot.current,msg);
	msg += sizeof(hw_info.drv_mot.current);
	MsgConvToData(&hw_info.drv_mot.freq,msg);
	msg += sizeof(hw_info.drv_mot.freq);
	MsgConvToData(&hw_info.drv_mot.mot_state,msg);
	msg += sizeof(hw_info.drv_mot.mot_state);

	MsgConvToData(&hw_info.steer_mot.pos,msg);
	msg += sizeof(hw_info.steer_mot.pos);
	MsgConvToData(&hw_info.steer_mot.duty,msg);
	msg += sizeof(hw_info.steer_mot.duty);
	MsgConvToData(&hw_info.steer_mot.current,msg);
	msg += sizeof(hw_info.steer_mot.current);
	MsgConvToData(&hw_info.steer_mot.freq,msg);
	msg += sizeof(hw_info.steer_mot.freq);
	MsgConvToData(&hw_info.steer_mot.mot_state,msg);
	msg += sizeof(hw_info.steer_mot.mot_state);
	
	MsgConvToData(&hw_info.fl_wheel_adc_val_sum,msg);
	msg += sizeof(hw_info.fl_wheel_adc_val_sum);
	MsgConvToData(&hw_info.fr_wheel_adc_val_sum,msg);
	msg += sizeof(hw_info.fr_wheel_adc_val_sum);
	MsgConvToData(&hw_info.fl_wheel_adc_count,msg);
	msg += sizeof(hw_info.fl_wheel_adc_count);
	MsgConvToData(&hw_info.fr_wheel_adc_count,msg);
	msg += sizeof(hw_info.fr_wheel_adc_count);	
	
	
	MsgConvToData(&hw_info.rr_wheel_encode_val,msg);
	msg += sizeof(hw_info.rr_wheel_encode_val);
	MsgConvToData(&hw_info.rl_wheel_encode_val,msg);
	msg += sizeof(hw_info.rl_wheel_encode_val);	

	hw_info_pub.publish(hw_info);
	
	//ROS_INFO("Rx MR base link hw info; tick=%d",hw_info.mr_tick);
}
/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> */
void DutMrMsg::ProcRxMrPosition(uint8_t const *msg, uint16_t const len)
{
	double x,y,theta,linear_x,angular_z;

	uint16_t len_t = 0;
	uint8_t *p;

	p 	= (uint8_t *)(&x);
	*p++= msg[len_t++];
	*p++= msg[len_t++];
	*p++= msg[len_t++];
	*p++= msg[len_t++];
	*p++= msg[len_t++];
	*p++= msg[len_t++];
	*p++= msg[len_t++];
	*p++= msg[len_t++];	

	p 	= (uint8_t *)(&y);
	*p++= msg[len_t++];
	*p++= msg[len_t++];
	*p++= msg[len_t++];
	*p++= msg[len_t++];
	*p++= msg[len_t++];
	*p++= msg[len_t++];
	*p++= msg[len_t++];
	*p++= msg[len_t++];	

	p 	= (uint8_t *)(&theta);
	*p++= msg[len_t++];
	*p++= msg[len_t++];
	*p++= msg[len_t++];
	*p++= msg[len_t++];
	*p++= msg[len_t++];
	*p++= msg[len_t++];
	*p++= msg[len_t++];
	*p++= msg[len_t++];	

	p 	= (uint8_t *)(&linear_x);
	*p++= msg[len_t++];
	*p++= msg[len_t++];
	*p++= msg[len_t++];
	*p++= msg[len_t++];
	*p++= msg[len_t++];
	*p++= msg[len_t++];
	*p++= msg[len_t++];
	*p++= msg[len_t++];
	
	p 	= (uint8_t *)(&angular_z);
	*p++= msg[len_t++];
	*p++= msg[len_t++];
	*p++= msg[len_t++];
	*p++= msg[len_t++];
	*p++= msg[len_t++];
	*p++= msg[len_t++];
	*p++= msg[len_t++];
	*p++= msg[len_t++];	

	mr_postion.x 			= x;
	mr_postion.y 			= y;
	mr_postion.theta 		= theta;
	mr_postion.linear_x 	= linear_x;
	mr_postion.angular_z 	= angular_z;


	ros::Time ros_time=ros::Time::now();

	tf::TransformBroadcaster mr_tf;
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(x,y,0));
	tf::Quaternion q;
	q.setRPY(0,0,theta);
	transform.setRotation(q);
	mr_tf.sendTransform( tf::StampedTransform(transform,ros_time,"Odom","dut_base") );
	
    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = ros_time;
    odom.header.frame_id = "Odom";
	
    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);;

    //set the velocity
    odom.child_frame_id = "dut_base";
    odom.twist.twist.linear.x = linear_x;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = angular_z;

    //publish the message
    odom_pub.publish(odom);
	
	//ROS_INFO("Rx MR positon msg,,,theta: %f",theta);
}


void DutMrMsg::PublishMrHwInfo(ros::Publisher &pub)
{
	
	//pub.publish(hw_info);
}





