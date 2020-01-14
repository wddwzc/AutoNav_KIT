#ifndef __MODEL_H
#define __MODEL_H
#include "pthread.h"
#include "motor.h"
#include "ros/ros.h"
#include "rs232.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "math.h"
#include "tf/transform_broadcaster.h"
#include "vehicle/sysparam.h"
#include "vehicle/Motorstate.h"

//#define USE_SIM  1

//#define WHEEL_DISTANCE (double)0.6        //轮间距
//#define ENCODER_THREADS      10000        //编码器线数
//#define REDUCTION_RATIO         30        //减速比
//#define WHEEL_DIAMETER (double)0.3        //
#define PI             (double)3.1415926
class Model{

public:
    Model();
    bool Init();
    ~Model(){
				cmdMotors(false);
				cmdMotors(false);
        rstMotors();
        delete m_Motors;
        delete m_rs232;
    }
    vehicle::sysparam sysParams;    //系统参数
    std::string serialPortName;         //与驱动器通信串口名称

    vehicle::Motorstate motorstate;
    int serialBaudrate;                      //串口通信波特率
    int Encoder_Threads;                 //编码器线数
    int Reduction_Ratio;                  //电机减速器减速比
    double Wheel_Diameter;             //驱动轮直径,单位m
    double Wheel_Distance;              //左右两侧驱动轮距离,单位m
    double MotorStatePubPeriod;
    bool useOdom;                             //是否使用里程计数据:true-发布里程计数据与变换;false-不发布
    int DriverID_FL;
    int DriverID_FR;
    int DriverID_BL;
    int DriverID_BR;
    pthread_mutex_t mutex_serial;
    int sample_interval;//
    RS232* m_rs232;
    ServoMotor*m_Motors;
    double v,w,tarV,tarW;
    double x,y,th;
    double RatioCountsPerMeter;
    pthread_t TID_Sample;
    ros::NodeHandle nh;
    ros::Subscriber m_cmdVelSub;
    ros::Publisher m_OdomPub;
    ros::Publisher m_infoPub;
    ros::Publisher m_motorStatePub;
    tf::TransformBroadcaster m_odomBroadcaster;

    bool cmdMotors(bool state);
    void rstMotors();
    void cmdVelSubCB(const geometry_msgs::TwistConstPtr&msg);
    void publishOdomnTransform();
    void VelRobot2Motor();
    void PosnVelMotor2Robot();
    void robotMove();
    void robotMove(double v,double w);
    static void* threadFuncSample(void*p);
    void startSample();
    void pulishMotorstate();
};
#endif

