#include "path_planner/global_path.h"
#include <math.h>
#include <cstring>
#include <fstream>
#include <stdlib.h>
#include <sstream>
#include <stack>
#include <vector>
#include <queue>
#include <algorithm>
#include <ros/ros.h>

global_path::global_path()
{
	INF = 100000.0;
	mode = 1;
	Snode = -1;
	Gnode = -1;
	current = -1;
	loadmap();
}

global_path::~global_path()
{
	for(int i = 0;i<nodenum;i++)
      delete [] map[i];
	delete [] map;
	
}

void global_path::updatekeypoint()
{
	if(!key_point.empty())
	{
		key_point.clear();
	}

	std::cout<<"global_path_nodesID "<<std::endl;

	for(int i=0;i<global_path_nodesID.size();i++)
	{
		std::cout<<global_path_nodesID[i]<<">>>>>> ";
		CPoint2d tem;
		int index = findindexfromID(global_path_nodesID[i]);
		tem.x = Nodeset[index].x;
		tem.y = Nodeset[index].y;
		key_point.push_back(tem);
	}

	std::cout<<std::endl;

	// for(int j=0;j<key_point.size();j++)
	// {
	// 	std::cout<<"key_point "<<key_point[j].x<<" "<<key_point[j].y<<std::endl;
	// }
}

void global_path::setSandGnode(int Snode,int Gnode)
{
	Snode = Snode;
	Gnode = Gnode;
}

void global_path::setCnode(int current)
{
	current = current;
}

std::vector<int> global_path::Dijkstra(int Snode,int Gnode)
{
	int *vist = new int[nodenum];
	int *path = new int[nodenum];
	float *lenth = new float[nodenum];

	for(int i=0;i<nodenum;i++)
	{
		vist[i] = 0;
	}

	for(int i=0;i<nodenum;i++)
	{
		path[i] = -1;
	}
	
	for(int i=0;i<nodenum;i++)
	{
        lenth[i]=INF;
    }
	
	int sindex = Snode - 1;
	int gindex = Gnode - 1;
	lenth[sindex]=0;
     
    vist[sindex]=1;      
	
	 for(int k=0;k<nodenum;k++)
         if(!vist[k]&&lenth[k]>(lenth[sindex]+map[sindex][k]))
         {
             lenth[k]=lenth[sindex]+map[sindex][k];
             path[k]=sindex;
         } 
		 path[sindex] = -1;
        //在起点已定情况下，初始化起点到各点的距离
    for(int k=0;k<nodenum;k++)
    {
        int min=INF,way;

        for(int i=0;i<nodenum;i++)
          if(!vist[i]&&lenth[i]<min)
          {
            min=lenth[i];
            way=i;
          }
            //寻找最短的可到达的未知点
          vist[way]=1;

         for(int i=0;i<nodenum;i++)
           if(!vist[i] && lenth[i]>(lenth[way]+map[way][i]))
            {
              lenth[i]=lenth[way]+map[way][i];
              path[i]=way;
			}
         //跟新到达各个点的距离数组
        if(way==gindex) break;
        //判断到达的点是否为终点
    }
	
    printf("minlen=%f\npath:\n",lenth[gindex]);       //通过path数组从终点到起点输出路径
        
	std::stack<int> pathnode;
	pathnode.push(gindex+1);
	while(path[gindex]>=0)
    {
            /*printf("%d<-%d\n",gindex+1,path[gindex]+1);*/
		printf("%d... ",path[gindex]+1);
		pathnode.push(path[gindex]+1);
        gindex=path[gindex];
    }
	std::cout<<std::endl;

	std::vector<int> Dijkstrapath;
	while(!pathnode.empty())
	{
		int tmp = pathnode.top();
		Dijkstrapath.push_back(tmp);
		std::cout<<tmp<<"--- ";
		pathnode.pop();
	}
	std::cout<<std::endl;
	
	delete []vist;
	delete []path;
	delete []lenth;
	
	return Dijkstrapath;
}

void global_path::DFS(int B)//深度遍历图
{
	path.push_back(B);//保存顶点值到路径中
	finalpath.push_back(B);
	NodeIDset.push(B);

	while(!NodeIDset.empty())
	{
		int temnode = NodeIDset.top();
		
		if(path.size()==14)
		{
			return;
		}

		int index =find(G.V.begin(),G.V.end(),temnode)-G.V.begin();

		int j = 0;
		bool added = false;

		int temdis = 99999;
		int temN = -1;
		int bestindex = -1;

		for(j=0;j<(int)G.V.size();j++)
		{
			if(find(path.begin(),path.end(),G.V[j])!=path.end())
				continue;
			if(G.E[index][j]  && G.E[index][j]!=INF /*&& temdis>G.E[index][j]*/)
			{
				added = true;
				NodeIDset.push(G.V[j]);
				path.push_back(G.V[j]);
				finalpath.push_back(G.V[j]);
				break;
			}	
		}

		if(!added)
		{
			NodeIDset.pop();
			int tem = NodeIDset.top();
			finalpath.push_back(tem);
		}
	}
}

void global_path::TravelallNodes(int current)
{
	int firstindex = -1;
	int lastindex = -1;
	firstindex =find(G.V.begin(),G.V.end(),current)-G.V.begin();
	
	DFS(current);

	for(int i=0;i<path.size();i++)
	{

		std::cout<<path[i]<<">>>>";
	}
	std::cout<<std::endl;

	//for(int j=0;j<finalpath.size();j++)
	//{
	//	std::cout<<finalpath[j]<<" ";
	//}

	lastindex = find(G.V.begin(),G.V.end(),finalpath[finalpath.size()-1])-G.V.begin();

	std::vector<int> tempath;
	if(G.E[lastindex][firstindex]  && G.E[lastindex][firstindex]!=INF)
	{
		finalpath.push_back(current);
	}
	else
	{
		int Snode = finalpath[finalpath.size()-1];
		int Gnode = current;
		tempath = Dijkstra(Snode,Gnode);
	}

	if(!tempath.empty())
	{
		for(int i=1;i<tempath.size();i++)
		{
			finalpath.push_back(tempath[i]);
		}
	}

	std::cout<<"final -------------------- "<<std::endl;	
	for(int j=0;j<finalpath.size();j++)
	{
		std::cout<<finalpath[j]<<" ";
	}
	std::cout<<std::endl;
}

//地图文件解析
void global_path::loadmap()
{

	// char* fileName_before = "/home/robot/Insrob_ws/map/topology_map.txt";
	char* fileName_before = "/home/robot/ROS_ws/map/topology_map.txt";
	
	Path_Node tmpnode;
	Edge tmpedge;

	char strFileID[128];
	char tmpchars[128];
	
	std::ifstream in(fileName_before);
	
	//read file id
	memset(tmpchars,0,sizeof(tmpchars));
	in>>tmpchars;

	std::cout<<tmpchars<<std::endl;

	if(strcmp(tmpchars,"FID"))
	{
		std::cout<<"FID read error "<<std::endl;
		return;
	}
		
	in>>strFileID;

	std::cout<<strFileID<<std::endl;
	
	memset(tmpchars, 0, sizeof(tmpchars));
	in>>tmpchars;

	std::cout<<tmpchars<<std::endl;

	if(strcmp(tmpchars, "NN"))
	{
		std::cout<<"Node read error "<<std::endl;
		return;
	}
	in>>nodenum;
	//cout<<nodenum<<endl;
	memset(tmpchars, 0, sizeof(tmpchars));
	in>>tmpchars;
	if(strcmp(tmpchars, "EN"))
	{
		std::cout<<"Edge read error "<<std::endl;
		return;
	}
	in>>edgenum;
	
	
	for(int i=0;i<nodenum;i++)
	{
		memset(tmpchars, 0, sizeof(tmpchars));
		in>>tmpchars>>tmpnode.nodeID>>tmpnode.type>>tmpnode.x
		>>tmpnode.y>>tmpnode.z>>tmpnode.yaw>>tmpnode.pitch>>tmpnode.roll
		>>tmpnode.gpsx>>tmpnode.gpsy>>tmpnode.gpsz>>tmpnode.gpsyaw
		>>tmpnode.odomx>>tmpnode.odomy>>tmpnode.odomz>>tmpnode.odomyaw
		>>tmpnode.gps_status>>tmpnode.gps_num>>tmpnode.latFlag
		>>tmpnode.lngFlag>>tmpnode.latValue>>tmpnode.lngValue>>tmpnode.altValue;
		Nodeset.push_back(tmpnode);
	}
	
	
	for(int j=0;j<edgenum;j++)
	{
		memset(tmpchars, 0, sizeof(tmpchars));
		in>>tmpchars>>tmpedge.nodeid1>>tmpedge.nodeid2
		>>tmpedge.dis>>tmpedge.odomdis>>tmpedge.flag;
		Edgeinfo.push_back(tmpedge);
	}
	
	memset(tmpchars, 0, sizeof(tmpchars));
	in>>tmpchars;
       
        
	if(strcmp(tmpchars, "GLOBAL-PATH"))
	{
		std::cout<<"GLOBAL-PATH read error "<<std::endl;
		return;
	}
	
	int tmpint;
	while(in>>tmpint)
	{
		global_path_nodesID.push_back(tmpint);
	}
	
	map = new float*[nodenum];
	for(int i=0;i<nodenum;i++)
	{
		map[i] = new float[nodenum];
	}
	
	for(int i=0;i<nodenum;i++)
	{
		for(int j=0;j<nodenum;j++)
		{
			if(i==j)
			{
				map[i][j] = 0;
			}
			
			map[i][j] = INF;
		}
	}
	
	for(int i =0;i<Edgeinfo.size();i++)
	{
		int index1 = Edgeinfo[i].nodeid1 - 1;
		int index2 = Edgeinfo[i].nodeid2 - 1;
		
		map[index1][index2] = Edgeinfo[i].dis;
		map[index2][index1] = Edgeinfo[i].dis;
	}
	
	//对遍历全节点的数据结构做初始化设置
	int *Idset = new int[nodenum];
	for(int i=0;i<Nodeset.size();i++)
	{
		Idset[i] = Nodeset[i].nodeID;
	}
	std::vector<int> V;
	std::vector<std::vector<int>>edges;
	for(int i=0;i<nodenum;i++)
	{
		V.push_back(Idset[i]);
	}
	
	delete []Idset;

	for(int j=0;j<nodenum;j++)
	{
		std::vector<int> temV;
		for(int i=0;i<nodenum;i++)
		{
			temV.push_back(map[j][i]);
		}
		edges.push_back(temV);
	}

	G.V=V;
	G.E=edges;
	
	
	/*switch (mode)
	{
		case 1 :{
			return;
			break;
		}
		case 2:{
			global_path_nodesID.clear();
			global_path_nodesID = Dijkstra(Snode,Gnode);
			break;
		}
		case 3:{
			global_path_nodesID.clear();
			TravelallNodes(current);
			global_path_nodesID = finalpath;
			break;
		}
		default:
		{
			return;
			break;
		}
	}*/

	updatekeypoint();
	
}

void global_path::Getshortpath(int mode,int Snode,int Gnode )
{
	global_path_nodesID.clear();
	global_path_nodesID = Dijkstra(Snode,Gnode);
	updatekeypoint();
}

void global_path::GetAlltravelpath(int mode,int current)
{
	global_path_nodesID.clear();
	TravelallNodes(current);
	global_path_nodesID = finalpath;
	updatekeypoint();
}

int  global_path::findindexfromID(int nodeid)
{
	int index = -1;
	for(int i =0;i<Nodeset.size();i++)
	{
		if(Nodeset[i].nodeID==nodeid)
		{
			index = i;
			return index;
		}
	}
	return index;
}


