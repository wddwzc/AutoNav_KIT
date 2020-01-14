#ifndef  global_path_H
#define  global_path_H
#include <iostream>
#include <vector>
#include <string>
#include <stack>
#include <ros/ros.h>
#include "point.h"

struct Path_Node
{
	int nodeID;    //???ID
	int type;      //???????,1???��??????  2??????????
	double x,y,z,yaw,pitch,roll;
	double gpsx,gpsy,gpsz,gpsyaw;
	double odomx,odomy,odomz,odomyaw;
	int gps_status, gps_num;
	int latFlag,lngFlag;
	double latValue,lngValue,altValue;
};

struct Edge
{
	int nodeid1;
	int nodeid2;
	float dis;
	float odomdis;
	int flag;
};

struct Graph
{
	std::vector<int>V;
	std::vector<std::vector<int>>E;
};

class global_path
{
	public:
	global_path();
	~global_path();	

	public:
	int mode; //ȫ��·���滮ģʽ 1 ���յ�ͼ·���� 2 ָ�����յ����·�� 3 �������нڵ�ص���ǰ�ڵ�
	int Snode,Gnode;
	int current;
	std::vector<Path_Node> Nodeset; //��ͼ�нڵ���Ϣ���
	std::vector<Edge> Edgeinfo; //��ͼ�б���Ϣ���
	std::vector<int> global_path_nodesID; //ָ����ȫ��·���ڵ�ID��ŵļ���
	
	float INF;
	float **map;
	int nodenum,edgenum;
	
	
	Graph G;
	std::vector<int>path;
	std::stack<int>NodeIDset;
	std::vector<int>finalpath;
	std::stack<int> Dijkstrapath;

	std::vector<CPoint2d> key_point;

	
	std::vector<int> Dijkstra(int Snode,int Gnode);
	void setSandGnode(int Snode,int Gnode);
	void setCnode(int current);
	void TravelallNodes(int Snode);
	void DFS(int B);
	void Getshortpath(int mode,int Snode,int Gnode);
	void GetAlltravelpath(int mode,int current);

	int findindexfromID(int nodeid);//�ӽڵ㼯�����ҵ�ȫ��·��ͨ���ڵ������
	void loadmap();

	void updatekeypoint();

};

#endif 