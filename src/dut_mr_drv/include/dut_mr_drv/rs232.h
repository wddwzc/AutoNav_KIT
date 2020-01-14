#ifndef __RS232_H
#define __RS232_H
#include "ros/ros.h"
#include "serial/serial.h"
#include "string"
#include "pthread.h"
class RS232{
public:
    serial::Serial* m_Ser;
    RS232();
    ~RS232(){
        delete m_Ser;
    }
    static bool m_configSerial(std::string portName,uint32_t baudrate,serial::Serial* ser);
    void m_sendMsg(uint8_t*pMsg,char eof);//
    void m_sendMsg(uint8_t*pMsg,uint8_t msgLen);
    bool m_WaitRes(std::string& res);
};

#endif

