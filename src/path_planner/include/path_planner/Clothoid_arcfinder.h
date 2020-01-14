#ifndef Clothoid_arcfinder_H
#define Clothoid_arcfinder_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/Point32.h"
#include "visualization_msgs/MarkerArray.h"
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include "tf/tf.h"
#include "nav_msgs/Path.h"

#include "geometry_msgs/PointStamped.h"


#include "geometry_msgs/Point32.h"
#include <iostream>
#include <stdlib.h>
#include "stdio.h"
#include <sys/time.h>
#include <string.h>
#include "math.h"
#include <fstream>
#include <vector>
#include <boost/heap/binomial_heap.hpp>
#include "path_planner/base.h"
#include "path_planner/point.h"


//class Visualize;

float toRadin1( float &degree);
float toDegree1( float &Radin);

//---pf
struct CompareNodes {
  /// Sorting 3D nodes by increasing C value - the total estimated cost
  bool operator()(const Node* lhs, const Node* rhs) const {
    return ((lhs->G)+(lhs->H)+(lhs->pf)) > ((rhs->G)+(rhs->H)+(rhs->pf));
  }

bool operator()(const Node2D* lhs, const Node2D* rhs) const {
    return ((lhs->G)+(lhs->H)) > ((rhs->G)+(rhs->H));
  }
};

/*
struct CompareNodes {
  /// Sorting 3D nodes by increasing C value - the total estimated cost
  bool operator()(const Node* lhs, const Node* rhs) const {
    return ((lhs->G)+(lhs->H)) > ((rhs->G)+(rhs->H));
  }
};*/




class Clothoid_arcfinder
{
public:

		 // tf::TransformListener listener_;

		//一个栅格360度角度划分
		const int D ;
		int deepth;
		float samplelength;
		float Curvmaxrate;
		float Curvminrate;
		float DCCurv;
		float Carwidth;
		float CarLength;
		int CObssize;
		float Max_Cruv;
		float Min_Cruv;

		float Minturning_radius;
		float CarCruv;
		
		float dgoal;
		float gama;
		float dobdis;
		float pama;
		
		//for show samplenodes
		//ros::Publisher pubNodes;
		//geometry_msgs::PoseArray poses3D;
		//void publishNode(Node* node);
                //void setpiblisher(ros::Publisher &pubNodes);

                //void setpublisher(Visualize &Visualiziation);
		
		
		Node StartNode;
		Node GoalNode;
		geometry_msgs::Pose startpoint;
		geometry_msgs::Pose goalpoint;

		CPoint2d Goalpoint;
      
		
		
        //栅格地图高宽
		int Map_With;    
		int Map_Height;
		//栅格地图分辨率
		float Map_Resolution;
		//栅格地图中心点
		geometry_msgs::Point Map_origin;
		
		// 地图里每个栅格的属性 -1无障碍 1是障碍
		std::vector<int> ObstacleIdx; 
		//表示每个栅格总的势场
		std::vector<float> Index_APF;
		//用于存储栅格是障碍栅格的索引
		std::vector<int> ObstacleIndex;
		
		std::vector<Node> final_path;
		std::vector<Node> Befinal_path;
        std::vector<Executepoint> controlpoints;
		
		std::vector<CPoint2d> m_refer_path;
		int refer_pathindex;
		CPoint2d lastpose;
		bool ChechReachG(const CPoint2d &Robpose);
		
		Clothoid_arcfinder();
		Clothoid_arcfinder(std::vector<CPoint2d> key_point);
		~Clothoid_arcfinder();

		void updateMap(const nav_msgs::OccupancyGrid& msg);
		void Setstarpoint(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& initial);
		void Setgoalpoint(const geometry_msgs::PoseStamped::ConstPtr& end);
		void SetStartNode_andGoalNode(const float sCurv);
		void setNodeset(Node* Nodeset);
		int setIdex(const Node* node);
		int setpfindex(Node* node);
		
		void updateH(Node* node);
		void updateG(Node* node);
		void updateNode_pf(Node *p);
		
		float ComputeAllR_field(const int &dx,const int &dy);
		float ComputeG_fieldinGrid(const int &Pdx,const int &Pdy);
		void setPointtoGrid(const geometry_msgs::Pose &point,int &Gdx,int &Gdy,int &Gindex);
		//float ComputeG_fieldinGrid(const int &Pdx,const int &Pdy);
		float ComputeR_fieldinGrid(const int &Pdx,const int &Pdy,const int obindex);
		float ComputeR_field(const int &pdx,const int &pdy);
		void updatepf(Node* node);
		void updateMapPf();
		void tracePath(const Node* node);
		
		bool existObs(const geometry_msgs::Point &CenterPoint);
		bool Reachgoal(const Node* BeCheckNode);
		void clothoid_arcfunction3(float &X,float &Y,const float &l,const float &A,const float &In_Curv);
		void clothoid_arcfunction4(float &X,float &Y,const float &l,const float &A,const float &In_Curv);
		void computeLine(float &X,float &Y,const float l);
		void computeArc(float &X,float &Y,const float l,const float &Curv_In);
		void connectclothoid_arcP_XY(float &X,float &Y, const float &sP_X,const float &sP_Y,const float &sP_Th,const float &l,const float &Curv_In,const float &Fial_Curv, const float &L);
		void clothoid_arcP_XY(float &EX,float &EY,float &ECurv,const float &sP_X,const float &sP_Y,const float &sP_Th,const float &Curv_In,const float &DetaCurv, float &Len);
		void Gen_clothoid_arcP_XY(float &EX,float &EY,float &ECurv,float &Clothoid_Len,const float &sP_X,const float &sP_Y,const float &sP_Th,const float &Curv_In,const float &DetaCurv, float &Len);
		float getNodeTh(const Node* Succ,const Node* nPred,const float &DetaCurv,const float &l);
		bool checkifObs(const Node*newNode,const float &DetaCurv,const float &l);
		Node* SearchAlgorithm(Node* Nodeset);
		
		bool make_plan();
		
		void SetControlpoints();

		float getDubinlenth(const Node* node);

		void reset2DNode(Node2D* node2Dset);

		int set2Didx(Node2D* node);

		void createSuccessor(Node2D* subnode,Node2D* node,int i);

		void updateG(Node2D* node);

		void updateH(Node2D* node);

		float getAstarlenth(const Node* node);

		void SetS_Gpoint(const float &SCruv,const float &Robx,const float &Roby,const float &Robth,const float &Goalx,const float &Goaly,const float &Goalth);

		
 		void Getsubgoal(std::string &frame_id,const CPoint2d &Robpose,CPoint2d &subgoal);
		
		void set_navpoint(std::vector<CPoint2d> key_point);

		void GetGoal(CPoint2d &goalpoint);
		
	
};
#endif 
