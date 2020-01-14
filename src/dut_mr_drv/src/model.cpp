#include "vehicle/model.h"

Model::Model():v(0),
    w(0),
    th(0),
    tarV(0),
    tarW(0),
    sample_interval(20)
{
    ros::NodeHandle private_nh("~");
    m_rs232=new RS232;
    m_Motors=new ServoMotor[4];
    mutex_serial=PTHREAD_MUTEX_INITIALIZER;
    m_OdomPub=nh.advertise<nav_msgs::Odometry>("odom",50);
    m_cmdVelSub=nh.subscribe("cmd_vel",10,&Model::cmdVelSubCB,this);
    m_infoPub=nh.advertise<vehicle::sysparam>("system_params",1,true);
    m_motorStatePub=nh.advertise<vehicle::Motorstate>("motor_state",10);

    private_nh.param(std::string("serialPortName"),serialPortName,std::string("/dev/ttyS1"));
    private_nh.param(std::string("serialBaudrate"),serialBaudrate,115200);
    private_nh.param(std::string("Wheel_Distance"),Wheel_Distance,0.7);
    private_nh.param(std::string("Wheel_Diameter"),Wheel_Diameter,0.3);
    private_nh.param(std::string("Encoder_Threads"),Encoder_Threads,10000);
    private_nh.param(std::string("Reduction_Ratio"),Reduction_Ratio,30);
    private_nh.param(std::string("sample_interval"),sample_interval,20);
    private_nh.param(std::string("useOdom"),useOdom,true);
    private_nh.param(std::string("DriverID_FR"),DriverID_FR,0);
    private_nh.param(std::string("DriverID_FL"),DriverID_FL,1);
    private_nh.param(std::string("DriverID_BR"),DriverID_BR,2);
    private_nh.param(std::string("DriverID_BL"),DriverID_BL,3);
    private_nh.param(std::string("MotorStatePubPeriod"),MotorStatePubPeriod,0.1);
    sysParams.serialPortName=serialPortName;
    sysParams.serialBaudrate=serialBaudrate;
    sysParams.Wheel_Distance=Wheel_Distance;
    sysParams.Encoder_Threads=Encoder_Threads;
    sysParams.Reduction_Ratio=Reduction_Ratio;
    sysParams.DriverID_FR=DriverID_FR;
    sysParams.DriverID_FL=DriverID_FL;
    sysParams.DriverID_BR=DriverID_BR;
    sysParams.DriverID_BL=DriverID_BL;
    m_infoPub.publish(sysParams);

    RatioCountsPerMeter=(double)(Encoder_Threads*Reduction_Ratio)/(Wheel_Diameter*PI);
    printf("\n");
    ROS_INFO("Parameter initialized:");
    ROS_INFO("RatioCountsPerMeter=%.4lf",RatioCountsPerMeter);
    ROS_INFO("serialPortName: %s",serialPortName.c_str());
    ROS_INFO("serialBaudrate=%d",serialBaudrate);
    ROS_INFO("Wheel_Distance: %.3lf",Wheel_Distance);
    ROS_INFO("Wheel_Diameter=%.3lf",Wheel_Diameter);
    ROS_INFO("Encoder_Threads=%d",Encoder_Threads);
    ROS_INFO("Reduction_Ratio=%d",Reduction_Ratio);
    printf("\n\n");
}

bool Model::Init(){
    if(!RS232::m_configSerial(serialPortName,9600,m_rs232->m_Ser)){
        return false;
    }
    m_Motors[0].ID=DriverID_FR;
    m_Motors[1].ID=DriverID_FL;
    m_Motors[2].ID=DriverID_BR;
    m_Motors[3].ID=DriverID_BL;
    uint8_t status=0;
    for(int i=3;i>=0;i--){
        m_Motors[i].m_rs232pt=m_rs232;
        if(m_Motors[i].setBaudrate(serialBaudrate))status++;
        usleep(59000);
    }
    if(status!=4)return false;
    usleep(2000000);
    m_rs232->m_Ser->close();
    usleep(1000000);
    if(!RS232::m_configSerial(serialPortName,serialBaudrate,m_rs232->m_Ser)){
        ROS_ERROR("Reopen serial %s failed,exit",m_rs232->m_Ser->getPort().c_str());
        return false;
    }
    sleep(3);
    return true;
}
bool Model::cmdMotors(bool state){
    std::string stateStr=(state)?("ENABLE"):("DISABLE");
    for(uint8_t i=0;i<=3;i++){
        if(m_Motors[i].motorCmd(state)){
            ROS_INFO("set Motor %d to %s succeeded",i,stateStr.c_str());
        }else{
            ROS_ERROR("set Motor %d to %s failed,exit",i,stateStr.c_str());
            return false;
        }
    }
    return false;
}
void Model::rstMotors(){
    for(int i=0;i>=0;i--){
        m_Motors[i].motorReset();
        usleep(50000);
    }
}
void Model::cmdVelSubCB(const geometry_msgs::TwistConstPtr&msg){
    tarV=msg->linear.x;
    tarW=msg->angular.z;
    robotMove();
}
void Model::publishOdomnTransform(){
    geometry_msgs::Quaternion odom_quat=tf::createQuaternionMsgFromYaw(th);
    //first, we'll publish the transform over tf
    ros::Time currTime=ros::Time::now();
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = currTime;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    m_odomBroadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = currTime;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = v;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = w;

    //publish the message
    m_OdomPub.publish(odom);
}

/*motor arrangement:
 * 1 0
 * 3 2
*/
void Model::VelRobot2Motor(){
    double WheelVL,WheelVR;
    WheelVL=tarV-tarW*Wheel_Distance/2;
    WheelVR=tarV+tarW*Wheel_Distance/2;
    m_Motors[0].tarV=WheelVR*RatioCountsPerMeter;//right
    m_Motors[1].tarV=-WheelVL*RatioCountsPerMeter;//left
    m_Motors[2].tarV=m_Motors[0].tarV;
    m_Motors[3].tarV=m_Motors[1].tarV;
}
void Model::PosnVelMotor2Robot(){
    static ros::Time lastTime=ros::Time::now();
    static int32_t mP[4]={0};
    int32_t nMP[4]={0};
    int32_t deltaMP[4]={0};

    for(uint8_t i=0;i<=3;i++){
        nMP[i]=m_Motors[i].pos;
        deltaMP[i]=nMP[i]-mP[i];
        mP[i]=nMP[i];
    }
    ros::Time currTime=ros::Time::now();
    double deltaTime=(currTime-lastTime).toSec();
    lastTime=currTime;

    double dL,dR;

    dL=-(double)(deltaMP[1]+deltaMP[3])/(RatioCountsPerMeter*2);
    dR=(double)(deltaMP[0]+deltaMP[2])/(RatioCountsPerMeter*2);

    double deltaX,deltaY,deltaTh;
    deltaX=(dL+dR)*cos(th)/2;
    deltaY=(dL+dR)*sin(th)/2;
    deltaTh=(dR-dL)/Wheel_Distance;
    ROS_DEBUG("dL=%.6lf,dR=%.6lf,deltaT=%.6lf,dx=%.6lf,dy=%.6lf,dth=%.6lf",dL,dR,deltaTime,deltaX,deltaY,deltaTh);
    x+=deltaX;y+=deltaY;th+=deltaTh;

    if(th>PI)th-=2*PI;
    if(th<-PI)th+=2*PI;
    v=(dL+dR)/(2*deltaTime);
    w=(dR-dL)/(Wheel_Distance*deltaTime);
}


void Model::robotMove(){
    VelRobot2Motor();
    pthread_mutex_lock(&mutex_serial);
    for(uint8_t i=0;i<=3;i++){
        m_Motors[i].setMotorVel(i,m_Motors[i].tarV*10);
    }
    pthread_mutex_unlock(&mutex_serial);
}
void Model::robotMove(double v, double w){
    tarV=v;tarW=w;
    ROS_INFO("tarV=%.4lf,tarW=%.4lf",tarV,tarW);
    robotMove();
}

void* Model::threadFuncSample(void*p){
    Model*pRobot=(Model*)p;
    ROS_INFO("Sampling thread started");
    while(ros::ok()){
        uint8_t statusP=0;
        pthread_mutex_lock(&(pRobot->mutex_serial));
        for(uint8_t i=0;i<=3;i++){
            if(pRobot->m_Motors[i].getMotorPos()){
                statusP++;
            }
        }
        pthread_mutex_unlock(&(pRobot->mutex_serial));
        if(statusP!=4)continue;
        pRobot->PosnVelMotor2Robot();               //update odom information
        if(pRobot->useOdom){
            pRobot->publishOdomnTransform();            //publish odom transform
        }
        pRobot->pulishMotorstate();
        double thd=180*(pRobot->th)/PI;
      //  ROS_INFO("x=%.4lf,y=%.4lf,thd=%.4lf",pRobot->x,pRobot->y,thd);
        usleep(pRobot->sample_interval*1000);
    }
}
void Model::pulishMotorstate(){
    motorstate.TS=ros::Time::now();
    for(uint8_t i=0;i<=3;i++){
        motorstate.state[i].ID=m_Motors[i].ID;
        motorstate.state[i].vel=m_Motors[i].vel;
        motorstate.state[i].pos=m_Motors[i].pos;
    }
    m_motorStatePub.publish(motorstate);
}
void Model::startSample(){
    pthread_create(&TID_Sample,NULL,threadFuncSample,this);//创建电机采样(里程计数据发布)线程
}
