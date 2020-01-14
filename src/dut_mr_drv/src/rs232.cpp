#include "ros/ros.h"
#include "dut_mr_drv/rs232.h"
#define MAX_MSG_LENGTH 100
//driver response data+'\r'
RS232::RS232(){
    m_Ser=new serial::Serial;
}

void RS232::m_sendMsg(uint8_t*pMsg,char eof){//
    if(!m_Ser->isOpen()){
        ROS_ERROR("Serial port not opened");
        return;
    }

//    printf("Ascii command: %s",(char*)pMsg);
//    printf("Hex format: ");
//    for(uint8_t i=0;i<=MAX_MSG_LENGTH;i++){
//        printf("%x ",pMsg[i]);
//        if(pMsg[i]=='\n')break;
//    }
//    printf("\n");

    uint8_t dataNum=0;
    for(dataNum=0;;dataNum++){
        if(m_Ser->write(&pMsg[dataNum],1)!=1){
            ROS_WARN("sending bytes overlapped");
            break;
        }
        if(pMsg[dataNum]==eof){
            ROS_DEBUG("EOF reached");
            break;
        }
    }
    dataNum++;
    ROS_DEBUG("Sending %d bytes",dataNum);
}
void RS232::m_sendMsg(uint8_t*pMsg,uint8_t msgLen){
    if(!m_Ser->isOpen()){
        ROS_ERROR("Serial port not opened");
        return;
    }
    m_Ser->write(pMsg,msgLen);
}

bool RS232::m_WaitRes(std::string& res){
    ros::Time before=ros::Time::now();
    m_Ser->readline(res,65535,std::string("\r"));

    double deltaT=(ros::Time::now()-before).toSec();
    uint8_t strLen=res.length();
    const char* strpt=res.c_str();
    char eoc=strpt[strLen-1];
    if((eoc!='\r')&&(eoc!='\n')){
       ROS_WARN("waiting for response timeout");
       return false;
    }else{
        ROS_DEBUG("response waited: %s",res.c_str());
    }
    return true;
}

bool RS232::m_configSerial(std::string portName,uint32_t baudrate,serial::Serial* ser){
    try{

        ser->setPort(portName);
        serial::Timeout to=serial::Timeout::simpleTimeout(10);
        ser->setTimeout(to);//
        ser->setBaudrate(baudrate);
        ser->open();
    }catch(serial::IOException &ex){
        ROS_ERROR("serial %s open failed, %s",portName.c_str(),ex.what());
        return false;
    }
    if(ser->isOpen()){
        ROS_INFO("serialport %s opened successfully,baudrate=%d",portName.c_str(),baudrate);
        ser->flush();
        return true;
    }else{
        ROS_ERROR("serial %s open failed",portName.c_str());
        return false;
    }
    return true;
}

