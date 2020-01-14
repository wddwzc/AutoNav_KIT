#include "vehicle/motor.h"
 //设置一个对象的数值
ServoMotor::ServoMotor():isSerialAssigned(false),pos(0),vel(0),lastPos(0){

}

uint8_t* ServoMotor::setObj(uint8_t nodeID,char memoryZone,std::string objID,int32_t value){
    static char pAsciiMsg[32];
    sprintf(pAsciiMsg,"%d s %c%s %d\n",nodeID,memoryZone,objID.c_str(),value);//节点地址,s,存储区,对象ID,数值
    return (uint8_t*)pAsciiMsg;
}
//获取一个对象的数值
uint8_t* ServoMotor::readObj(uint8_t nodeID,char memoryZone,std::string objID){  //获取一个对象的数值
    static char pAsciiMsg[32];
    sprintf(pAsciiMsg,"%d g %c%s\n",nodeID,memoryZone,objID.c_str());//节点地址,g,存储区,对象ID
    return (uint8_t*)pAsciiMsg;
}


//v 数值[回车]
bool ServoMotor::extractValue(const char* AsciiMsg,int32_t*dataValue){
    uint8_t msgLen=strlen(AsciiMsg);
    char*dataAscii=new char[msgLen-2];
    if((AsciiMsg[0]=='v')&&(AsciiMsg[1]==' ')&&((AsciiMsg[msgLen-1]=='\n')||(AsciiMsg[msgLen-1]=='\r'))){
        memcpy(dataAscii,AsciiMsg+2,msgLen-2);
        dataAscii[msgLen-3]='\0';
        *dataValue=atoi(dataAscii);
        delete dataAscii;
    }else{
        ROS_ERROR("responded data format wrong");
        return false;
    }
    return true;
}
//mode:0-表示等待一个应答信号"ok";1-表示等待一个对象变量的值
bool ServoMotor::sendCmdnWaitRes(uint8_t*pMsg,uint8_t mode,int32_t* value){
    std::string strMsg;
    ros::Time initTime=ros::Time::now();
    m_rs232pt->m_sendMsg(pMsg,'\n');
    bool status=m_rs232pt->m_WaitRes(strMsg);
    dataEchoTime=(ros::Time::now()-initTime).toSec()*1000;//测量串口数据发送到返回耗时,单位ms
    if(status){
        ROS_DEBUG("Echo waited");
        if(mode==0){
            if(strMsg!=RIGHT_REPLY){
                ROS_ERROR("not match");
                return false;
            }
        }else{
            if(!extractValue(strMsg.c_str(),value))return false;
        }
    }else{
        return false;
    }
    return true;
}

bool ServoMotor::setWorkMode(uint8_t nodeID,uint8_t workMode){
    uint8_t*pMsg=setObj(nodeID,'r',OBJ_WORKMODE,workMode);
    bool state=sendCmdnWaitRes(pMsg,0,NULL);
    if(state){
        ROS_DEBUG("set Motor %d workmode to %d",nodeID,workMode);
    }else{
        ROS_ERROR("set Motor %d workmode to %d failed",nodeID,workMode);
    }
    return state;
}
bool ServoMotor::setAccelLimit(uint8_t nodeID,uint32_t value){
    uint8_t*pMsg=setObj(nodeID,'r',OBJ_ACCEL_LM,value);
    return sendCmdnWaitRes(pMsg,0,NULL);
}
bool ServoMotor::setDccelLimit(uint8_t nodeID,uint32_t value){
    uint8_t*pMsg=setObj(nodeID,'r',OBJ_DCCEL_LM,value);
    return sendCmdnWaitRes(pMsg,0,NULL);
}
bool ServoMotor::setMotorVel(uint8_t nodeID,int32_t value){
    uint8_t*pMsg=setObj(nodeID,'r',OBJ_TAR_VEL,value);
    bool state=sendCmdnWaitRes(pMsg,0,NULL);
    if(state){
        ROS_DEBUG("set Motor %d velocity succeeded",nodeID);
    }else{
        ROS_ERROR("set Motor %d velocity failed",nodeID);
    }
    return state;
}


bool ServoMotor::setBaudrate(uint8_t nodeID,uint32_t baudrate){
    uint8_t*pMsg=setObj(nodeID,'r',OBJ_BAUD_RATE,baudrate);
    m_rs232pt->m_sendMsg(pMsg,'\n');
    return true;
}
bool ServoMotor::resetDriver(uint8_t nodeID){
    char pMsg[10];
    sprintf(pMsg,"%d r\n",nodeID);
    m_rs232pt->m_sendMsg((uint8_t*)pMsg,'\n');
    return true;
}

bool ServoMotor::motorCmd(bool state){
    if(state)
        return setWorkMode(ID,WORK_MODE_VEL);
    else
        return setWorkMode(ID,WORK_MODE_STOP);
}
bool ServoMotor::getMotorPos(){
    uint8_t*pMsg=readObj(ID,'r',OBJ_ACT_POS);
    lastPos=pos;                //前一次的位置
    bool state=sendCmdnWaitRes(pMsg,1,&pos);//新的位置
    ros::Time currTime=ros::Time::now();
    double deltaT=currTime.toSec()-updateTime;
    updateTime=currTime.toSec();
    if(state){
        if(deltaT!=0)vel=(pos-lastPos)/deltaT;//计算速度
        ROS_DEBUG("get Motor %d pos succeeded,pos=%d",ID,pos);
    }else{
        ROS_ERROR("get Motor %d pos failed",ID);
    }
    return state;
}
bool ServoMotor::getMotorVel(){
    uint8_t*pMsg=readObj(ID,'r',OBJ_ACT_VEL);
    bool state=sendCmdnWaitRes(pMsg,1,&vel);
    if(state){
        ROS_DEBUG("get Motor %d vel succeeded,pos=%d",ID,vel);
    }else{
        ROS_ERROR("get Motor %d vel failed",ID);
    }
    return state;
}
bool ServoMotor::setMotorVel(){
    return setMotorVel(ID,tarV);
}
bool ServoMotor::setMotorVel(int32_t vel){
    return setMotorVel(ID,vel);
}
bool ServoMotor::resetDriver(){
    return resetDriver(ID);
}
bool ServoMotor::setBaudrate(uint32_t baudrate){
    return setBaudrate(ID,baudrate);
}
void ServoMotor::motorReset(){
    uint8_t pMsg[5];
    sprintf((char*)pMsg,"%d r\n",ID);
     m_rs232pt->m_sendMsg(pMsg,'\n');
}

