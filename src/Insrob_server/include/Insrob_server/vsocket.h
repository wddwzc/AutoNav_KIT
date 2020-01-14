#ifndef VSOCKET_H
#define VSOCKET_H
#include "Insrob_server/global.h"
#include "Insrob_server/vpoint.h"
using namespace std;

#define BUF_SIZE 128

class VSocket
{
public:
    VSocket();
    ~VSocket();
public:
    bool Start();//?????
    bool Accept();//?socket?accept??
    CPackage RecvData();//????
    bool SendPos(char *pointpose,const int length);//??????
    bool SendPC(char *pointcloud,const int length);//??????
    void Terminate();//????
    void close_current_sock();

    bool isPointSaveServer;
    //bool isXYZ;
    bool isPoseSaveServer;
    bool isSaveNavMap;
    bool isPointTransBack;
    bool isPoseTransBack;
    bool isPointTransSparse;
    string current_path;
    string priorfilename;
public:
    bool m_bissuccessInit;//?????????
    bool m_is_locate;//?????????,????Accept?????
private:
    int recv_large(int len, char* buffer);
    void recv_prior_map_file(int len);//??????????????
//  bool ForbidenBlock();//???
    //void createDir(const char* path);
private:
    char *buffer;
    int serv_sock;//soceket for listenning 
    int clnt_sock;//socket for transmission
//  int m_iLen;//????
};

#endif // VSOCKET_H
