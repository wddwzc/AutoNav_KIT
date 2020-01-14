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
	int type;      //???????,1???λ??????  2??????????
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
	int mode; //全局路经规划模式 1 按照地图路线走 2 指定起终点最短路径 3 遍历所有节点回到当前节点
	int Snode,Gnode;
	int current;
	std::vector<Path_Node> Nodeset; //地图中节点信息组合
	std::vector<Edge> Edgeinfo; //地图中边信息组合
	std::vector<int> global_path_nodesID; //指定的全局路径节点ID编号的集合
	
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

	int findindexfromID(int nodeid);//从节点集合中找到全局路径通过节点的索引
	void loadmap();

	void updatekeypoint();

};

#endif 