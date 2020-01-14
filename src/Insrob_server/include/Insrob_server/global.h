#ifndef GLOBAL_H
#define GLOBAL_H

#include <iostream>
#include <fstream>
#include <vector>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <errno.h>
#include <netdb.h>

#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

using namespace std;

enum State{
    ROBOT_STATIC = 0,
    ROBOT_NORMAL,
    ROBOT_HIGHEST
};

enum CmdType
{
    CMD_TYPE_DATA = 0,           // request data
    CMD_DATA_GETCLOUD,
    CMD_DATA_GETPOSE,
    CMD_DATA_ENDCLOUD,
    CMD_DATA_ENDPOSE,
    CMD_DATA_GETALL,
    CMD_DATA_ENDALL,
	CMD_DATA_PATHNODE,

    CMD_TYPE_MOVEMENT,
    CMD_MOVE_LEFTFORWARD,       // forward to left
    CMD_MOVE_FORWARD,               // go straight
    CMD_MOVE_RIGHTFORWARD,      // forward to right
    CMD_MOVE_LEFT,                          // turn left
    CMD_MOVE_STOP,                      // stop
    CMD_MOVE_RIGHT,                     // turn right
    CMD_MOVE_LEFTBACK,              // back to left
    CMD_MOVE_BACK,                      // back off
    CMD_MOVE_RIGHTBACK,              // back to right

    CMD_TYPE_INSTRUCTION,
    CMD_INSTRUCTION_INTERRUPT,
    CMD_INSTRUCTION_MANUAL,
    CMD_INSTRUCTION_NAVIGATION,
    CMD_INSTRUCTION_MAPPING,

    CMD_NONE,
    CMD_OFF,
    CMD_MOVE_KEEP
};

//接收的数据包头的结构
struct CPackage {
    CmdType msgType;
    CmdType msgData;
    unsigned int msgLen;
};

/*
enum CmdType{
    //停止并关闭套接字
    CMD_ROBOT_OFF = 0,
    //暂停
    CMD_ROBOT_PAUSE,
    //空挡，类似于暂停状态，但仅仅是车辆停在原处
    CMD_ROBOT_STAY,

    //一般速度前进后退，左右直转
    CMD_ROBOT_UP,
    CMD_ROBOT_BACK,
    CMD_ROBOT_LEFT,
    CMD_ROBOT_RIGHT,

    //加速状态前进后退，左右直转
    CMD_ROBOT_ACC_UP,
    CMD_ROBOT_ACC_BACK,
    CMD_ROBOT_ACC_LEFT,
    CMD_ROBOT_ACC_RIGHT,

    //发生错误
    CMD_ROBOT_ERROR
};
*/

#endif // GLOBAL_H
