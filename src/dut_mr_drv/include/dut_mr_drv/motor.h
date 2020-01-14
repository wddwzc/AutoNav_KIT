#ifndef __MOTOR_H
#define __MOTOR_H
#include "ros/ros.h"
#include "rs232.h"
#include "string"
#include "stdio.h"
#include "stdlib.h"
#include "unistd.h"
#define RIGHT_REPLY              "ok\r"



//********************电机变量宏定义*********************
#define OBJ_WORKMODE        "0x24"      //工作模式(写入)
#define OBJ_TAR_VEL         "0x2f"      //速度模式下目标运动速度(写入)
#define OBJ_ACT_POS         "0x17"      //电机实际位置(读取)
#define OBJ_ACT_VEL         "0x18"      //电机实际velocity(读取)
#define OBJ_ACCEL_LM        "0x36"      //加速度限制(写入)
#define OBJ_DCCEL_LM        "0x37"      //减速度限制(写入)
#define OBJ_EMGSTOP_VEL     "0x39"      //急速停止速率(写入)
#define OBJ_BAUD_RATE       "0x90"      //设置波特率

#define UNIT_ADCCELRATION   1000        //value&1000=counts/sec^2
#define UNIT_VELOCITY       0.1         //value*0.1=counts/sec
class ServoMotor{
public:
    typedef enum{
        WORK_MODE_VEL=11,
        WORK_MODE_STOP=0
    }WorkMode;

    int32_t pos;
    int32_t lastPos;
    int32_t vel;
    int32_t tarV;
    double dataEchoTime;       //驱动器数据 回传时间
    double updateTime;
    uint8_t ID;
    ServoMotor();
    RS232* m_rs232pt;
    bool isSerialAssigned;
    static uint8_t* setObj(uint8_t nodeID,char memoryZone,std::string objID,int32_t value);            //设置一个对象的数值
    static uint8_t* readObj(uint8_t nodeID,char memoryZone,std::string objID);  //获取一个对象的数值
    bool extractValue(const char* AsciiMsg,int32_t*dataValue);
    bool sendCmdnWaitRes(uint8_t*pMsg,uint8_t mode,int32_t* value);
    bool setWorkMode(uint8_t nodeID,uint8_t workMode);

    bool setAccelLimit(uint8_t nodeID,uint32_t value);
    bool setDccelLimit(uint8_t nodeID,uint32_t value);
    bool setMotorVel(uint8_t nodeID,int32_t value);
    bool setBaudrate(uint8_t nodeID,uint32_t baudrate);
    bool resetDriver(uint8_t nodeID);

    bool motorCmd(bool state);
    bool getMotorPos();
    bool getMotorVel();
    bool setMotorVel();
    bool setMotorVel(int32_t vel);
    bool resetDriver();
    bool setBaudrate(uint32_t baudrate);
    void motorReset();
};
#endif

