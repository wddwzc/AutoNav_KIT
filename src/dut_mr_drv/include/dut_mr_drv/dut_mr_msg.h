#ifndef __DUT_MR_MSG_H
#define __DUT_MR_MSG_H


#include "dut_mr_drv/rs232.h"
#include "dut_mr_drv/MrHwInfo.h"
#include <nav_msgs/Odometry.h>
#include "dut_mr_drv/RawMotorCmd.h"

#include <boost/thread/thread.hpp> 

/*
01: tick msg, 		ROS->MR
02: control msg, 	ROS->MR

*/
typedef enum
{
	UART_MSG_WAIT_START,
    UART_MSG_RECV_NORMAL,
    UART_MSG_RECV_ESCAPE
}UART_MSG_RECV_STATE;

typedef struct
{
	uint8_t msg_key[8];
	uint8_t msg_key_len;
	uint32_t msg_min_len;
	void (*process)(void *p,uint8_t const *msg, uint16_t const len);
}UART_CMD_PROCESS;

class DutMrMsg
{
	public:
		RS232* rs232;
		DutMrMsg();
		~DutMrMsg(){
			delete rs232;
		}

		void SetMrStop()	{ ROS_INFO("DUT MR Require Stop ...");req_linear_x=0,req_angular_z=0; req_twist_msg = false;};
		void SetMrSpeed(double linear_x,double angular_z);
		void SetMrRawMotorCmd(const dut_mr_drv::RawMotorCmd::ConstPtr &cmd);
		void GetMrPosition(double &x,double &y, double &theta,double &linear_x,double &angular_z);
		

		void McuTickMsgRx(uint8_t const *msg, uint16_t const len);		//01 01
		void ProcRxMsgMrHwInfo(uint8_t const *msg, uint16_t const len);	//02 01
		void ProcRxMrPosition(uint8_t const *msg, uint16_t const len);	//03 01 
		
	private:
		bool 	mr_twist_ctrl;
		bool  	req_twist_msg;

		dut_mr_drv::RawMotorCmd raw_motor_cmd;
	


		int32_t steer_mot_pos;
		//int16_t drv_mot_pwm;
		
		double 	req_linear_x,req_angular_z;
		
		struct
		{
			double x,y,theta,linear_x,angular_z;
		}mr_postion;

		
		ros::Publisher hw_info_pub;

		uint8_t uart_rx_buf[1024];
		size_t  uart_rx_len;

		UART_MSG_RECV_STATE	uart_msg_rcv_state;
	 	uint8_t				uart_msg_crc;
	 	uint16_t			uart_msg_len;
		uint8_t uart_msg_cmd_buf[256];


		uint32_t heartbeat_tick;


		ros::Publisher odom_pub;
		boost::mutex mutex_cmd;
		
		
		void UartRxMsgInfoOut(void);
		void UartDecodeRxMsgInfoOut();
		
		void UartRxMsgParse(void);
		void UartTxMsgCmd(uint8_t code,uint8_t sub_code,uint8_t *cmd,uint8_t cmd_len);

		void PublishMrHwInfo(ros::Publisher &);


		void MsgConvToData(uint32_t *ptr,uint8_t const*p )	{uint8_t * desc = (uint8_t*)ptr;*desc ++ = *p++;*desc ++ = *p++;*desc ++ = *p++;*desc ++ = *p++;}
		void MsgConvToData(int32_t *ptr,uint8_t const*p )	{uint8_t * desc = (uint8_t*)ptr;*desc ++ = *p++;*desc ++ = *p++;*desc ++ = *p++;*desc ++ = *p++;}
		void MsgConvToData(float *ptr,uint8_t const*p )		{uint8_t * desc = (uint8_t*)ptr;*desc ++ = *p++;*desc ++ = *p++;*desc ++ = *p++;*desc ++ = *p++;}		
		void MsgConvToData(uint16_t *ptr,uint8_t const*p )	{uint8_t * desc = (uint8_t*)ptr;*desc ++ = *p++;*desc ++ = *p++;}
		void MsgConvToData(int16_t *ptr,uint8_t const*p )	{uint8_t * desc = (uint8_t*)ptr;*desc ++ = *p++;*desc ++ = *p++;}
		/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> */
		// void* RxMsgThreadHandle(){	/*((DutMrMsg*)p)->RxUartMsgThread();*/}
		//static void* TxMsgThreadHandle(void*p){	((DutMrMsg*)p)->TxUartMsgThread();}
		void RxUartMsgThread(void);
		void TxUartMsgThread(void);
		

		
		static void McuTickMsgRx_s(void *p,uint8_t const *msg, uint16_t const len) 				{((DutMrMsg*)p)->McuTickMsgRx(msg,len);}
		static void ProcRxMsgMrHwInfo_s(void *p,uint8_t const *msg, uint16_t const len) 		{((DutMrMsg*)p)->ProcRxMsgMrHwInfo(msg,len);}
		static void ProcRxMrPosition_s(void *p,uint8_t const *msg, uint16_t const len)		 	{((DutMrMsg*)p)->ProcRxMrPosition(msg,len);}
		UART_CMD_PROCESS uart_cmd_proc_tbl[3] =
			{ 
				{{0x01,0x01},2,0,McuTickMsgRx_s},			//MR->HOST
				{{0x02,0x01},2,0,ProcRxMsgMrHwInfo_s},
				{{0x03,0x01},2,0,ProcRxMrPosition_s},		//MR->HOST
				
			};
				
};

#endif

