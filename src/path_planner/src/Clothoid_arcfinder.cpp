#include "path_planner/Clothoid_arcfinder.h"
#include "path_planner/base.h"
#include "path_planner/dubins.h"
#include "math.h"
#include "path_planner/Bspline.h"	
#include "path_planner/smooth.h"
#include "path_planner/computetool.h"

extern ros::Publisher pubNodes;
geometry_msgs::PoseArray poses3D;

int Maxsize = 100;

bool testdata = true;
int Dubin = 3;

void publishNode(Node* node)
{
	poses3D.header.stamp = ros::Time::now();
 	poses3D.header.frame_id = "/camera_init";
	geometry_msgs::Pose pose;
	pose.position.x = node->x;
	pose.position.y = node->y;
	pose.position.z = 0;
	pose.orientation = tf::createQuaternionMsgFromYaw(node->Th);
	// if(poses3D.poses.size()>100)
	// {
	// 	poses3D.poses.clear();

	// }
	poses3D.poses.push_back(pose);
	pubNodes.publish(poses3D);
}


float toDegree1( float &Radin)
{
	float Degree;
	Degree = 180 / M_PI * Radin;
	if(Degree>=-180 && Degree<180)
	{
		return Degree;
	}
	else
	{ 
		if(Degree>0)
		{
			float DegreeYu = fmod(Degree,360.0);
			if(DegreeYu>=180)
			{
				return (DegreeYu-360.0);
			}
			else
			{
				return DegreeYu;
			
			}
		}
		else
		{
			float DegreeYu = fmod(Degree,360.0);
			if(DegreeYu<-180)
			{
				return (DegreeYu+360.0);
			}
			else
			{
				return DegreeYu;
			
			}
		}
	}
}


float toRadin1( float &degree)
{
	float Radin;
	if(degree>=-180 && degree<180)
	{ 
		Radin = degree * M_PI/180;
		return (Radin);
	}
	else
	{
		if(degree>0)
		{
			float DegreeYu = fmod(degree,360.0);
			if(DegreeYu>=180)
			{
				 Radin = (DegreeYu-360.0)* M_PI/180;
				 return (Radin);
			}
			else
			{
				Radin = DegreeYu * M_PI/180;
				return (Radin);
			
			}
		}
		else
		{
			float DegreeYu = fmod(degree,360.0);
			if(DegreeYu<-180)
			{
				Radin = (DegreeYu+360.0)* M_PI/180;
				return (Radin);
			}
			else
			{
				Radin = DegreeYu * M_PI/180;
				return (Radin);
			
			}
		}
	}
}


Clothoid_arcfinder::Clothoid_arcfinder():D(36)
{
	//pubNodes = n.advertise<geometry_msgs::PoseArray>("/visualizeNodes3DPoses", 100);
	
	samplelength = 1.0;

	deepth = 0;

	Curvmaxrate = 1.0;
	Curvminrate = -1.0;
	DCCurv = 0.2;
	Max_Cruv = 1.0;
	Min_Cruv = -1.0;

	CarLength = 0.8;
	Carwidth = 0.8;


	
	
	Minturning_radius = 1/Max_Cruv;
	CarCruv = 0.0;
	
	
	dgoal = 10.0;
	gama = 0.5;
	dobdis = 1.2;
	pama = 1000;
	
	
	refer_pathindex = 5;
       
}


Clothoid_arcfinder::Clothoid_arcfinder(std::vector<CPoint2d> key_point):D(36)
{
	samplelength = 1.0;

	deepth = 0;

	Curvmaxrate = 1.0;
	Curvminrate = -1.0;
	DCCurv = 0.2;
	Max_Cruv = 1.0;
	Min_Cruv = -1.0;

	CarLength = 0.8;
	Carwidth = 0.8;


	
	
	Minturning_radius = 1/Max_Cruv;
	CarCruv = 0.0;
	
	
	dgoal = 10.0;
	gama = 0.5;
	dobdis = 1.2;
	pama = 1000;
	
	
	refer_pathindex = 5;

	set_navpoint(key_point);
}

Clothoid_arcfinder::~Clothoid_arcfinder()
{
	
	
}


void Clothoid_arcfinder::set_navpoint(std::vector<CPoint2d> key_point)
{
	if(!m_refer_path.empty())
	{
		m_refer_path.clear();
	}

	// std::cout<<"Clothoid_arcfinder "<<std::endl;
	// for(int u=0;u<key_point.size();u++)
	// {
	// 	std::cout<<"key_point "<<key_point[u].x<<" "<<key_point[u].y<<std::endl;
	// }

	std::vector<CPoint2d> allpath;

	for(int i=1;i<key_point.size();i++)
	{
		allpath.push_back(key_point[i-1]);
		float th = atan2((key_point[i].y - key_point[i-1].y),(key_point[i].x - key_point[i-1].x));
		float dis = key_point[i].Dist(key_point[i-1]);

		CPoint2d newpoint;
		newpoint.x = key_point[i-1].x + dis/3*cos(th);
		newpoint.y = key_point[i-1].y + dis/3*sin(th);
		allpath.push_back(newpoint);

		CPoint2d newpoint1;
		newpoint1.x = key_point[i-1].x + 2*dis/3*cos(th);
		newpoint1.y = key_point[i-1].y + 2*dis/3*sin(th);
		allpath.push_back(newpoint1);

	}

	allpath.push_back(key_point[key_point.size()-1]);

	// std::cout<<"allpath "<<std::endl;
	// for(int i=0;i<allpath.size();i++)
	// {
	// 	std::cout<<allpath[i].x<<" "<<allpath[i].y<<std::endl;
	// }


	Bspline LBSpline;

	LBSpline.ShapePoints = allpath;

	std::vector<CPoint2d> smoothpath;

	LBSpline.GetNodeVector();

	LBSpline.GetControlPnts();

	// for(int j=0;j<LBSpline.ControlPoints.size();j++)
	// {
		
	// 	std::cout<<"ControlPoints "<<LBSpline.ControlPoints[j].x<<" "<<LBSpline.ControlPoints[j].y<<std::endl;
	// }
	

	LBSpline.GetBSplinePnts();

	smoothpath = LBSpline.BSplinePoints;

	for(int i=1;i<smoothpath.size();i++)
	{
		float Th = atan2((smoothpath[i].y-smoothpath[i-1].y),(smoothpath[i].x-smoothpath[i-1].x));
		smoothpath[i-1].pointangle = Th;
	}

	smoothpath[smoothpath.size()-1].pointangle = smoothpath[smoothpath.size()-2].pointangle;

	CPoint2d lastpoint;
	bool first = false;

	for(int j=0;j<smoothpath.size();j++)
	{
		CPoint2d tempose;
		 		
		if(!first)
		{
			lastpoint = smoothpath[j];
			tempose.x = smoothpath[j].x;
		 	tempose.y = smoothpath[j].y;
		 	tempose.pointangle = smoothpath[j].pointangle;
		 	m_refer_path.push_back(tempose);
		 	first = true;
		}
		else
		{
			float dis = smoothpath[j].Dist(lastpoint);

			if(dis>=1.0)
			{
				lastpoint = smoothpath[j];
				tempose.x = smoothpath[j].x;
		 		tempose.y = smoothpath[j].y;
				tempose.pointangle = smoothpath[j].pointangle;
		 		m_refer_path.push_back(tempose);
			}
		}
	}

	CPoint2d tempose;
	tempose.x = smoothpath[smoothpath.size()-1].x;
	tempose.y = smoothpath[smoothpath.size()-1].y;
	tempose.pointangle = smoothpath[smoothpath.size()-1].pointangle;
	m_refer_path.push_back(tempose);

	Goalpoint = tempose;

}


void Clothoid_arcfinder::SetS_Gpoint(const float &SCruv,const float &Robx,const float &Roby,const float &Robth,const float &Goalx,const float &Goaly,const float &Goalth)
{

	StartNode.x = Robx;
	StartNode.y = Roby;
	StartNode.Th = Robth;
	StartNode.Cruv = SCruv;
	StartNode.Cruvrate = 0;
	StartNode.G = 0;
	StartNode.H = 0;
	StartNode.parent = nullptr;
	StartNode.ol = false;
	StartNode.cl = false;
	StartNode.Clothoid_Len = 0;

	// float  BemodifyGoalx = Goalx;
	// float  BemodifyGoaly = Goaly;

	// float Max_X = Map_origin.x + Map_With * Map_Resolution;
	// float Max_Y = Map_origin.y + Map_Height * Map_Resolution;

	// if(BemodifyGoalx<Map_origin.x && BemodifyGoaly<Map_origin.y)
	// {
	// 	std::cout<<"BemodifyGoalx "<<"BemodifyGoaly "<<std::endl;
	// 	BemodifyGoalx = Map_origin.x;
	// 	BemodifyGoaly = Map_origin.y;
	// }

	// if(BemodifyGoalx<Map_origin.x && BemodifyGoaly>Map_origin.y)
	// {
	// 	std::cout<<"BemodifyGoalx "<<std::endl;
	// 	BemodifyGoalx = Map_origin.x;
	// }

	// if(BemodifyGoalx>Map_origin.x && BemodifyGoaly<Map_origin.y)
	// {
	// 	std::cout<<"BemodifyGoaly "<<std::endl;
	// 	BemodifyGoaly = Map_origin.y;
	// }


	// if(BemodifyGoalx>Max_X && BemodifyGoaly<Max_Y)
	// {
	// 	std::cout<<"Max_X "<<std::endl;
	// 	BemodifyGoalx = Max_X;
	// }

	// if(BemodifyGoalx<Max_X && BemodifyGoaly>Max_Y)
	// {
	// 	std::cout<<"Max_Y "<<std::endl;
	// 	BemodifyGoaly = Max_Y;
	// }

	// if(BemodifyGoalx>Max_X && BemodifyGoaly>Max_Y)
	// {
	// 	std::cout<<"Max_Y  Max_Y "<<std::endl;
	// 	BemodifyGoaly = Max_Y;
	// 	BemodifyGoalx = Max_X;
	// }




	GoalNode.x = Goalx;
	GoalNode.y = Goaly;
	GoalNode.Th = Goalth;
	GoalNode.Cruv = 0;
	GoalNode.Cruvrate = 0;
	GoalNode.G = 0;
	GoalNode.H = 0;
	GoalNode.parent = nullptr;
	GoalNode.ol = false;
	GoalNode.cl = false;
}



void Clothoid_arcfinder::reset2DNode(Node2D* node2Dset)
{
	for(int j=0;j<Map_Height;j++)
		for(int i=0;i<Map_With;i++)
		{
			int index = i+j*Map_With;
			node2Dset[index].ol = false;
			node2Dset[index].cl = false;
			if(ObstacleIdx[index] == 1)
			{
				node2Dset[index].obd = 1;
			}
			else
			{
				node2Dset[index].obd = -1;
			}
		}
}
	
int Clothoid_arcfinder::set2Didx(Node2D* node)
{
	int dx =  floor((node->x - Map_origin.x)/Map_Resolution);
	int dy  =  floor((node->y - Map_origin.y)/Map_Resolution);
	int index = dy * Map_With + dx;
    return index;
}
	
const float dx[] = {-1,-1,0,1,1,1,0,-1};
const float dy[] = {0,1,1,1,0,-1,-1,-1};

void Clothoid_arcfinder::createSuccessor(Node2D* subnode, Node2D* node,int i)
{/*
	subnode->x = node->x + dx[i]; 
	subnode->y = node->y + dy[i];
	//subnode->parent = node;*/
}

void Clothoid_arcfinder::updateG(Node2D* node)
{
	Node2D* prenode;
    prenode = node->parent;
	node->G = prenode->G + (sqrt(((node->x) - (prenode->x))*((node->x) - (prenode->x))+ ((node->y) - (prenode->y))*((node->y) - (prenode->y))));
}
	
void Clothoid_arcfinder::updateH(Node2D* node)
{
	node->H = sqrt((node->x - GoalNode.x)*(node->x - GoalNode.x)+(node->y-GoalNode.y)*(node->y-GoalNode.y));	
}
	
	
float Clothoid_arcfinder::getAstarlenth(const Node* node)
{
	Node2D* node2Dset = new Node2D[Map_With * Map_Height]();
	reset2DNode(node2Dset);
	
	boost::heap::binomial_heap<Node2D*,boost::heap::compare<CompareNodes>> O;
	
	Node2D start,goal;
	start.x = node->x;
	start.y = node->y;
	start.G = 0;
	start.H = 0;
	start.ol = false;
	start.cl = false;
	start.parent = nullptr;
	
	
	
	goal.x = goalpoint.position.x;
	goal.y = goalpoint.position.y;
	
	start.ol = true;
	
	int iPred = set2Didx(&start);
	int igoal = set2Didx(&goal);
	int iSucc;
	
	node2Dset[iPred] = start;
	
	//std::cout<<"iPred "<<iPred<<" "<<igoal<<std::endl;
	
	O.push(&start);
	
	Node2D* nPred; //= nullptr;
  	Node2D* nSucc; //= nullptr;
	
	
	//std::cout<<"O.szie() "<<O.size()<<std::endl;
	
	 while (!O.empty())
	 {
		 nPred = O.top();
		 
		 //std::cout<<"--------------- O.size()------- "<<O.size()<<std::endl;
		 iPred = set2Didx(nPred);
		 
		 if(node2Dset[iPred].cl) 
		 {
      		// pop node from the open list and start with a fresh node
      		O.pop();
      		continue;
    	 }
		 else
		 {
			 if(node2Dset[iPred].ol)
			 {
				node2Dset[iPred].cl = true;
				node2Dset[iPred].ol = false;	
				O.pop(); 
				if(iPred == igoal)
				{
					std::cout<<"reach 2D 2D "<<std::endl;
					return (nPred->G);
					delete []node2Dset;
				}
				else
				{
					//std::cout<<"2D search "<<std::endl;
					
					
					for (int i = 0;i<8;i++)
					{
						//std::cout<<"nPred "<<nPred->x<<" "<<nPred->y<<std::endl;
						
						//std::cout<<"dx[i] "<<i<<" "<<dx[i]<<" "<<dy[i]<<std::endl;
						
						Node2D temnode;
						
						temnode.x = nPred->x + dx[i];
						
						temnode.y = nPred->y + dy[i];
						
						temnode.parent = nPred;
						
						nSucc = &temnode;
						
						//nSucc->x = nPred->x + dx[i];
						
						//nSucc->y = nPred->y + dy[i];
						
						//std::cout<<"nSucc "<<nSucc->x<<" "<<nSucc->y<<std::endl;
						
						//createSuccessor(nSucc,nPred,i);
						
						//nSucc->parent = nPred;
						
						//std::cout<<"generate node "<<std::endl;
						
						Node2D* Parent;
						
						Parent = nSucc->parent;
						
						//std::cout<<"parent "<<Parent->x<<" "<<Parent->y<<std::endl;
						//std::cout<<"nSucc "<<nSucc->x<<" "<<nSucc->y<<" "<<nSucc->parent<<std::endl;
						
						iSucc = set2Didx(nSucc);
						
						if((iSucc>=0 && iSucc<Map_With * Map_Height) && (node2Dset[iSucc].obd!=1) && (!node2Dset[iSucc].cl))
						{
							updateG(nSucc);
							
							if((nSucc->G)<(node2Dset[iSucc].G) || (!node2Dset[iSucc].ol))
							{
								updateH(nSucc);
								nSucc->ol = true;
								nSucc->cl =false;
								node2Dset[iSucc] = *(nSucc);
								O.push(&node2Dset[iSucc]);
								nSucc = nullptr;
							}
							else
							{
								//std::cout<<"nSucc delete 1"<<std::endl;
								nSucc = nullptr;
							}//if((nSucc->G)<(node2Dset[iSucc].G) || (!node2Dset[iSucc].ol))
						}
						else
						{
							//std::cout<<"nSucc delete 2"<<std::endl;
							nSucc = nullptr;
						}
					}
				}//if(iPred == igoal) else....
			 }//if(node2Dset[iPred].ol) no else
		 }//if (node2Dset[iPred].cl) 
	 }//while (!O.empty())
	std::cout<<"jump circle "<<std::endl;
	delete []node2Dset;
}


float Clothoid_arcfinder::getDubinlenth(const Node* node)
{
	//std::cout<<"dubin dubin "<<std::endl;
	
	//float goalth = tf::getYaw(goalpoint.orientation);
	double q0[] = {node->x,node->y,node->Th};
    double q1[] = {GoalNode.x,GoalNode.y,GoalNode.Th};
		
	
	DubinsPath path;
		
	dubins_init(q0,q1,Minturning_radius,&path);
	
	float len = dubins_path_length(&path);
	
	//std::cout<<"LENNNN "<<len<<std::endl;
		
	
	return len;
	
}

/*void Clothoid_arcfinder::setpublisher(Visualize &Visualiziation)
{
	Visualiziation = Visualiziation;

}
*/
/*void Clothoid_arcfinder::publishNode(Node* node)
{      
  	poses3D.header.stamp = ros::Time::now();
   	poses3D.header.frame_id = "/map";

	geometry_msgs::Pose pose;
	pose.position.x = node->x;
	pose.position.y = node->y;
	pose.position.z = 0;
	pose.orientation = tf::createQuaternionMsgFromYaw(node->Th);
	poses3D.poses.push_back(pose);
	pubNodes.publish(poses3D);
}*/


void Clothoid_arcfinder::Setstarpoint(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& initial)
{
	if(testdata)
	{
	startpoint.position.x = initial->pose.pose.position.x;
    startpoint.position.y = initial->pose.pose.position.y;
    startpoint.position.z = initial->pose.pose.position.z;
    startpoint.orientation = initial->pose.pose.orientation;
	}
}

void Clothoid_arcfinder::Setgoalpoint(const geometry_msgs::PoseStamped::ConstPtr& end)
{
	if(testdata)
	{
	goalpoint.position.x = end->pose.position.x;
    goalpoint.position.y = end->pose.position.y;
    goalpoint.position.z = end->pose.position.z;
    goalpoint.orientation = end->pose.orientation;
	}
}


void Clothoid_arcfinder::SetStartNode_andGoalNode(const float sCurv)
{
	if(testdata)
	{
	StartNode.x = startpoint.position.x;
	StartNode.y = startpoint.position.y;
	StartNode.Th = tf::getYaw(startpoint.orientation);
	StartNode.Cruv = sCurv;
	StartNode.Cruvrate = 0;
	StartNode.G = 0;
	StartNode.H = 0;
	StartNode.parent = nullptr;
	StartNode.ol = false;
	StartNode.cl = false;
	StartNode.Clothoid_Len = 0;
	}

	if(!testdata)
	{
	StartNode.x = 12.1376;
	StartNode.y = 1.01352;
	StartNode.Th = 1.52441;
	StartNode.Cruv = 0;
	StartNode.Cruvrate = 0;
	StartNode.G = 0;
	StartNode.H = 0;
	StartNode.parent = nullptr;
	StartNode.ol = false;
	StartNode.cl = false;
	StartNode.Clothoid_Len = 0;
	}
	
	if(testdata)
	{
	GoalNode.x = goalpoint.position.x;
	GoalNode.y = goalpoint.position.y;
	GoalNode.Th = tf::getYaw(goalpoint.orientation);
	GoalNode.Cruv = 0;
	GoalNode.Cruvrate = 0;
	GoalNode.G = 0;
	GoalNode.H = 0;
	GoalNode.parent = nullptr;
	GoalNode.ol = false;
	GoalNode.cl = false;
	}
	
	if(!testdata)
	{
	GoalNode.x = 12.3337;
	GoalNode.y = 24.1796;
	GoalNode.Th = 1.54987;
	GoalNode.Cruv = 0;
	GoalNode.Cruvrate = 0;
	GoalNode.G = 0;
	GoalNode.H = 0;
	GoalNode.parent = nullptr;
	GoalNode.ol = false;
	GoalNode.cl = false;
	GoalNode.Clothoid_Len = 0;
	}
}


void Clothoid_arcfinder::setNodeset(Node* Nodeset)
{
	//std::ofstream  of;
    //of.open("/home/ds/judgedat.txt",std::ios::app|std::ios::out);
	
	for(int i= 0;i<deepth;i++)
	{
		Nodeset[i].cl = false;
		Nodeset[i].ol = false;
		Nodeset[i].obd =-1;
	}

    for(int i= 0;i<deepth;i++)
    {
       //of<<" ******** "<<i<<" "<<i%D<<" ";

	   int TwoDIdx = (i-i%D)/D;

       if(ObstacleIdx[TwoDIdx]==1)
       {
         // of<<" TwoDIdx "<<TwoDIdx<<"---";
        //std::cout<<" TwoDIdx "<<TwoDIdx<<"---";
		//for(int j =0;j<D;j++)
       // {
             // of<<i<<" ";
            //std::cout<<TwoDIdx+j<<" ";
             Nodeset[i].obd = 1;
        //}
       }
      //std::cout<<std::endl;
     //of<<std::endl;
    }
    //of.close();
}

int Clothoid_arcfinder::setpfindex(Node* node)
{
	int dx =  floor((node->x - Map_origin.x)/Map_Resolution);
	int dy  =  floor((node->y - Map_origin.y)/Map_Resolution);
	int pfindex = dy * Map_With + dx;
    return pfindex;
}

int Clothoid_arcfinder::setIdex(const Node* node)
{
	float nodeth = node->Th;
	
	/*std::cout<<"Debug----setIdex() node->x "<<node->x<<std::endl;
	std::cout<<"Debug----setIdex() node->y "<<node->y<<std::endl;
	std::cout<<"Debug----setIdex() node->Th "<<node->Th<<std::endl;*/
	
    float nodeangle = toDegree1(nodeth);
    if(nodeangle == 180)
    {
       nodeangle = -180.0;
    }
    
	//std::cout<<"Debug----setIdex() nodeangle "<<nodeangle<<std::endl;
	
	
	int dx =  floor((node->x - Map_origin.x)/Map_Resolution);
	int dy  =  floor((node->y - Map_origin.y)/Map_Resolution);
	int dth = floor(nodeangle/10) + 18;
	
	// std::cout<<"Debug----setIdex() dx "<<dx<<std::endl;
	// std::cout<<"Debug----setIdex() dy "<<dy<<std::endl;
	// std::cout<<"Debug----setIdex() dth "<<dth<<std::endl;
	
	int index = (dx + Map_With * dy)*D + dth;
	return index;
}

void Clothoid_arcfinder::updateH(Node* node)
{
	
	// std::cout<<"node "<<node->x<<" "<<node->y<<std::endl;
 //    std::cout<<"GoalNode "<<GoalNode.x<<" "<<GoalNode.y<<std::endl;
	
	switch(Dubin)
	{
		case 1:
		{
			float Dubinpathlen = getDubinlenth(node);

			std::cout<<"Dubinpathlen "<<Dubinpathlen<<std::endl;

			node->H = Dubinpathlen;

			node->H = 100 * node->H;
			break;
		}
		case 2:
		{
			float Dubinpathlen = getDubinlenth(node);

			std::cout<<"Dubinpathlen "<<Dubinpathlen<<std::endl;

		
	
			float ManthenH = sqrt((node->x - GoalNode.x)*(node->x - GoalNode.x)+(node->y-GoalNode.y)*(node->y-GoalNode.y));
			
			node->H = Dubinpathlen;
			if(ManthenH>Dubinpathlen)
			{
				node->H = ManthenH;
			}
			node->H = 100 * node->H;
			break;
		}
		case 3:
		{
			
			float ManthenH = sqrt((node->x - GoalNode.x)*(node->x - GoalNode.x)+(node->y-GoalNode.y)*(node->y-GoalNode.y));
			node->H = ManthenH;
			node->H = 100 * node->H;
			break;
		}
		
		case 4:
		{
			float ManthenH = sqrt((node->x - GoalNode.x)*(node->x - GoalNode.x)+(node->y-GoalNode.y)*(node->y-GoalNode.y));
			float Dubinpathlen = getDubinlenth(node);
			node->H = ManthenH + Dubinpathlen;
			node->H = 100 * node->H;
			break;
		}
	}
}

void Clothoid_arcfinder::updateG(Node* node)
{
    Node* prenode;
    prenode = node->parent;
	
	node->G = prenode->G + 100 * (1 + sqrt((node->Cruvrate) * (node->Cruvrate)));
		
    //node->G = prenode->G + 100 * (sqrt(((node->x) - (prenode->x))*((node->x) - (prenode->x))+ ((node->y) - (prenode->y))*((node->y) - (prenode->y))) + sqrt((node->Cruv - ((node->parent)->Cruv)) * (node->Cruv - ((node->parent)->Cruv))));

	//node->G = prenode->G + 100 * (sqrt(((node->x) - (prenode->x))*((node->x) - (prenode->x))+ ((node->y) - (prenode->y))*((node->y) - (prenode->y))));
	
	//node->G = 0;
}

void Clothoid_arcfinder::setPointtoGrid(const geometry_msgs::Pose &point,int &Gdx,int &Gdy,int &Gindex)
{
	Gdx = floor((point.position.x - Map_origin.x)/Map_Resolution);
	Gdy = floor((point.position.y - Map_origin.y)/Map_Resolution);
	Gindex = Gdy*Map_With + Gdx;
}

float Clothoid_arcfinder::ComputeR_fieldinGrid(const int &Pdx,const int &Pdy,const int obindex)
{
	int obdx,obdy;
	obdx = obindex%Map_With;
	obdy = obindex/Map_With;
	
	float R_field = 0.0;
	float X_Sq = (obdx - Pdx)*(obdx - Pdx)*Map_Resolution*Map_Resolution;
	float Y_Sq = (obdy - Pdy)*(obdy - Pdy)*Map_Resolution*Map_Resolution;
	
	float distoOb = sqrt(X_Sq+Y_Sq);
	
	if(distoOb<dobdis)
	{
		R_field = 0.5 * pama * (1/distoOb-1/dobdis)*(X_Sq+Y_Sq);
		
	}
	else
	{
		R_field = 0;	
	}
	return R_field;
}

float Clothoid_arcfinder::ComputeG_fieldinGrid(const int &Pdx,const int &Pdy)
{
	int Gdx,Gdy,Gindex;
	setPointtoGrid(goalpoint,Gdx,Gdy,Gindex);
	
	float G_field = 0.0;
	float X_Sq = (Gdx - Pdx)*(Gdx - Pdx)*Map_Resolution*Map_Resolution;
	float Y_Sq = (Gdy - Pdy)*(Gdy - Pdy)*Map_Resolution*Map_Resolution;
	float distogoal = sqrt(X_Sq+Y_Sq);
	
	if(distogoal<dgoal)
	{
		G_field = 0.5*gama*(X_Sq+Y_Sq);
	}
	else
	{
		G_field =  dgoal*gama*distogoal-0.5*gama*dgoal*dgoal;
	}
	
	//std::cout<<"G_field "<<G_field<<std::endl;
	return G_field;
}

float Clothoid_arcfinder::ComputeR_field(const int &pdx,const int &pdy)
{
	int CRObsize = dobdis/Map_Resolution;
	float AllR_field = 0;
	
	for(int i = (-CRObsize);i<=CRObsize;i++)
	{
		for(int j = (-CRObsize);j<=CRObsize;j++)
		{
			float temAllR_field = 0;
			
			int index = (pdx+j)+(pdy+i)*Map_With;

			if(index<0 || index>=ObstacleIdx.size())
			{
				continue;
			}
			if(ObstacleIdx[index] == 1)
			{
				temAllR_field = ComputeR_fieldinGrid(pdx,pdy,index);	
			}
			if(temAllR_field>AllR_field)
			{
				AllR_field = temAllR_field;
			}
		}
	}
	
	return AllR_field;
}

void Clothoid_arcfinder::updateNode_pf(Node *p)
{
	float R_field = 0;
	float G_field = 0;
	int pdx = floor((p->x - Map_origin.x)/Map_Resolution);
    int pdy = floor((p->y - Map_origin.y)/Map_Resolution);
	
	//G_field = ComputeG_fieldinGrid(pdx,pdy);
	R_field = ComputeR_field(pdx,pdy);
	
	// if(R_field >0)
	// {

	
	// 	std::cout<<" ----- R_field ----- "<<R_field<<std::endl;
	
	// }

	p->pf = (G_field+R_field);
	
}

///--pf
void Clothoid_arcfinder::updatepf(Node* node)
{
	int dx =  floor((node->x - Map_origin.x)/Map_Resolution);
	int dy  =  floor((node->y - Map_origin.y)/Map_Resolution);
	int pfindex = dy * Map_With + dx;
	node->pf = Index_APF[pfindex];
}

float Clothoid_arcfinder::ComputeAllR_field(const int &dx,const int &dy)
{
	float AllR_field = 0;
	for(int i=0;i<ObstacleIndex.size();i++)
	{
		float temAllR_field = 0;
		
		temAllR_field = ComputeR_fieldinGrid(dx,dy,ObstacleIndex[i]);
		
		if(temAllR_field>AllR_field)
		{
			AllR_field = temAllR_field;
		}
		//AllR_field = std::max(temAllR_field,AllR_field);
	}
	return AllR_field;
}

/*float Clothoid_arcfinder::ComputeG_fieldinGrid(const int &Pdx,const int &Pdy)
{
	int Gdx,Gdy,Gindex;
	setPointtoGrid(goalpoint,Gdx,Gdy,Gindex);
	float G_field = 0.0;
	
	float X_Sq = (Gdx - Pdx)*(Gdx - Pdx)*Map_Resolution*Map_Resolution;
	float Y_Sq = (Gdy - Pdy)*(Gdy - Pdy)*Map_Resolution*Map_Resolution;
	float distogoal = sqrt(X_Sq+Y_Sq);
	if(distogoal<dgoal)
	{
		G_field = 0.5*gama*(X_Sq+Y_Sq);
	}
	else
	{
		G_field =  dgoal*gama*distogoal-0.5*gama*dgoal*dgoal;
	}
	
	//std::cout<<"G_field "<<G_field<<std::endl;
	return G_field;
}*/


void Clothoid_arcfinder::updateMapPf()
{
	Index_APF.assign(Map_Height*Map_With,-1);
	// std::cout<<"Map_Height*Map_With "<<Map_Height*Map_With<<std::endl;
	
	// std::ofstream ofile;
	//记录斥力势场
	//std::ofstream ofile1;
	// ofile.open("/home/ds/G_field.txt",std::ios::app|std::ios::out);
	//ofile1.open("/home/robot/R_field.txt",std::ios::trunc|std::ios::out);
	
    float R_field = 0;
	float G_field = 0;
	
	// std::cout<<"ObstacleIdx.size()< "<<ObstacleIdx.size()<<std::endl;
	
	for(int j=0;j<Map_Height;j++)
		for(int i=0;i<Map_With;i++)
		{
			int temindex = i+j*Map_With;
			// std::cout<<"temindex "<<temindex<<std::endl;
			
			if(ObstacleIdx[temindex]== 1)
			{
				//G_field = ComputeG_fieldinGrid(i,j,goalpoint);
				R_field = 800;
				Index_APF[temindex] = R_field ;//+ G_field;
			}
			else
			{
				R_field = ComputeAllR_field(i,j);
				// G_field = ComputeG_fieldinGrid(i,j);
				// Index_APF[temindex] = R_field + G_field;
			}
			// ofile<<i<<" "<<j<<" "<<G_field<<std::endl;
			//ofile1<<(150-j)<<" "<<(150-(150 - i))<<" "<<R_field<<std::endl;
		}
	
		//for(int j=0;j<Index_APF.size();j++)
		//{
			//int dethadx,dethady;
			//dethadx = j%Map_With;
		    //dethady = j/Map_With;
			//std::cout<<dethadx<<" "<<dethady<<" "
			//std::cout<<Index_APF[j]<<std::endl;
		//}

		//std::cout<<"end -------------------- "<<std::endl;	
}

void Clothoid_arcfinder::updateMap(const nav_msgs::OccupancyGrid& msg)
{
   ObstacleIdx.clear();
   Index_APF.clear();
   ObstacleIndex.clear();
   
   Map_With = msg.info.width;
   Map_Height = msg.info.height;
   Map_origin = msg.info.origin.position;
   Map_Resolution = msg.info.resolution;
	
   //std::cout<<"map set "<<Map_origin.x<<" "<<Map_origin.y<<" "<<Map_Resolution<<" "<<Map_With<<" "<<Map_Height<<std::endl;

   deepth =  Map_With * Map_Height * D;

   ObstacleIdx.assign(Map_With * Map_Height, -1);
   
   for(int i=0;i<(msg.info.width)*(msg.info.height);++i)
   {
       //printf("%d\n",i);
       //printf("%d\n",(unsigned char)msg.data[i]);
       // std::cout<<" i "<<i<<" "<<msg.data[i]<<std::endl;
       //if((map->data[i])==100)
   		//printf("%d ",(unsigned char)(msg.data[i]));


       if((unsigned char)(msg.data[i]) >=120)
        {
           // std::cout<<" i "<<i<<" ";
           // printf("%d ",(unsigned char)(msg.data[i]));

           // int x = i/150;
           // int y = i%150;
           // float x1 = x * 0.1 - 7.5;
           // float y1 = y * 0.1 - 7.5;

           // std::cout<<x<<" "<<y<<" "<<x1<<" "<<y1<<"-------- ";
 
           //msg.data[i]<<std::endl;
		   ObstacleIdx[i] = 1; 
	   	   ObstacleIndex.push_back(i);
        }
   }

   std::cout<<std::endl;

   // updateMapPf();

   	// updateMapPf(ObstacleIdx,Index_APF);

   //std::cout<<"ObstacleIdx "<<ObstacleIdx.size()<<std::endl;
   
   //std::cout<<"ObstacleIndex "<<ObstacleIndex.size()<<std::endl;
   
   //update all map proteinal fields
  /*struct timeval start;
	
	struct timeval timeend;
	
	gettimeofday(&start, NULL); 			// get the beginning time
		
	updateMapPf(ObstacleIdx,Index_APF);
   
    std::cout<<"----------------------------"<<std::endl;
	
	gettimeofday(&timeend, NULL);  				// get the end time
    long long total_time = (timeend.tv_sec - start.tv_sec) * 1000000 + (timeend.tv_usec - start.tv_usec); // get the run time by microsecond
	printf("total time is %lld us\n", total_time);
	total_time /= 1000; 					// get the run time by millisecond
   	printf("total time is %lld ms\n", total_time);
		
	std::ofstream ofile2;
	ofile2.open("/home/ds/All_field.txt",std::ios::app|std::ios::out);
	for(int j=0;j<Index_APF.size();j++)
	{
		int dethadx,dethady;
		dethadx = j%Map_With;
		dethady = j/Map_With;
		ofile2<<dethadx<<" "<<dethady<<" "<<Index_APF[j]<<std::endl;
		std::cout<<dethadx<<" "<<dethady<<" "<<Index_APF[j]<<std::endl;
	}*/

   //std::cout<<"update map finish --- "<<std::endl;
}

bool Clothoid_arcfinder::Reachgoal(const Node* BeCheckNode)
{
	float GoalvectX = cos(GoalNode.Th);
	float GoalvectY = sin(GoalNode.Th);
	
	//std::cout<<"GoalNode "<<GoalNode.x<<" "<<GoalNode.y<<" "<<GoalNode.Th<<std::endl;
	//std::cout<<" GoalvectX "<<GoalvectX<<" GoalvectY " <<GoalvectY<<std::endl;
	
	float VerticalGX;
	float VerticalGY;
	
	VerticalGX = -(GoalvectY/sqrt(GoalvectX*GoalvectX+GoalvectY*GoalvectY));
	VerticalGY = (GoalvectX/sqrt(GoalvectX*GoalvectX+GoalvectY*GoalvectY));
	
	
	//std::cout<<" VerticalGX "<<VerticalGX<<" VerticalGY " <<VerticalGY<<std::endl;
	
	float toGdisvectX;
	float toGdisvectY;
	
	toGdisvectX = (BeCheckNode->x - GoalNode.x);
	toGdisvectY = (BeCheckNode->y - GoalNode.y);
	
	//std::cout<<" toGdisvectX "<<toGdisvectX<<" toGdisvectY " <<toGdisvectY<<std::endl;
	
	float CostoGth;
	float toGth;
	
	float toGdisvectlen = sqrt(toGdisvectX*toGdisvectX+toGdisvectY*toGdisvectY);

	float VerticalGlen = sqrt(VerticalGX*VerticalGX+VerticalGY*VerticalGY);
	
	//std::cout<<" toGdisvectlen "<<toGdisvectlen<<" VerticalGlen "<<VerticalGlen<<std::endl;
	//std::cout<<(toGdisvectX*VerticalGX + VerticalGY*toGdisvectY)<<std::endl;
	
	CostoGth = (toGdisvectX*VerticalGX + VerticalGY*toGdisvectY)/(toGdisvectlen*VerticalGlen);
	
	//std::cout<<" CostoGth "<<CostoGth<<std::endl;
	
	toGth = acos(CostoGth);
	
	//std::cout<<" toGth "<<toGth<<std::endl;
	
	float DistoGoal;
	
	DistoGoal = sqrt((BeCheckNode->x - GoalNode.x)*(BeCheckNode->x - GoalNode.x)+(BeCheckNode->y - GoalNode.y)*(BeCheckNode->y - GoalNode.y));
	
	std::cout<<" reach goal check..... "<<BeCheckNode->x<<" "<<BeCheckNode->y<<" "<<(DistoGoal*sin(toGth))<<" "<<sqrt((BeCheckNode->Th - GoalNode.Th)*(BeCheckNode->Th - GoalNode.Th))<<std::endl;
	
	if((DistoGoal*sin(toGth))<=1.0 && sqrt((BeCheckNode->Th - GoalNode.Th)*(BeCheckNode->Th - GoalNode.Th))<=0.14)
	{
		std::cout<<"reach goal "<<std::endl;
		return true;
	}
	return false;
}

void Clothoid_arcfinder::clothoid_arcfunction3(float &X,float &Y,const float &l,const float &A,const float &In_Curv)
{
	X = l-l*l*l/6* In_Curv* In_Curv-l*l*l*l* In_Curv/(8*A)-l*l*l*l*l/(40*A*A);
	Y = l*l* In_Curv/2+l*l*l/(6*A)- In_Curv* In_Curv* In_Curv* In_Curv*l*l*l*l/24-l*l*l*l*l* In_Curv* In_Curv/(20*A)-l*l*l*l*l*l* In_Curv/(48*A*A)-l*l*l*l*l*l*l/(336*A*A*A);
}

void Clothoid_arcfinder::clothoid_arcfunction4(float &X,float &Y,const float &l,const float &A,const float &In_Curv)
{
	X = l-l*l*l/6*In_Curv*In_Curv+l*l*l*l*In_Curv/(8*A)-l*l*l*l*l/(40*A*A);
	Y = l*l*In_Curv/2-l*l*l/(6*A)-In_Curv*In_Curv*In_Curv*In_Curv*l*l*l*l/24+l*l*l*l*l*In_Curv*In_Curv/(20*A)-l*l*l*l*l*l*In_Curv/(48*A*A)+l*l*l*l*l*l*l/(336*A*A*A);
}

void Clothoid_arcfinder::connectclothoid_arcP_XY(float &X,float &Y, const float &sP_X,const float &sP_Y,const float &sP_Th,const float &l,const float &Curv_In,const float &Fial_Curv, const float &L)
{
		float DetaCurv;
		float A1,A2;
		float l_1,l_2;
		

		float chpoint_x;
		float chpoint_y;
		//float chpoint_Curv;
		float chpoint_Th;

		DetaCurv = (Fial_Curv-Curv_In)/L;
		l_1 = (0-Curv_In)/DetaCurv;
		l_1 = sqrt(l_1*l_1);
	
		//chpoint_Curv = Curv_In+l_1*DetaCurv;

		l_2 = L-l_1;

		A1 = l_1/(0-Curv_In);
		A1 = sqrt(A1*A1);

		A2 = l_2/Fial_Curv;
		A2 = sqrt(A2*A2);

		float nowCurv = Curv_In+l*DetaCurv;

		if(Curv_In>0 )       //----+  ---- 0ÇúÏß
		{
			clothoid_arcfunction4(chpoint_x,chpoint_y,l_1,A1,Curv_In);
			//changepoint.y =changepoint.y*(-1);		

	
			float G_chpoint_x;
			float G_chpoint_y;

			G_chpoint_x = sP_X + chpoint_x*cos(sP_Th)-chpoint_y*sin(sP_Th);
	        G_chpoint_y = sP_Y + chpoint_x*sin(sP_Th)+chpoint_y*cos(sP_Th);
			float l_11 =  l_1 - 0.1;

		
			float anochpoint_x;
			float anochpoint_y;
	

			clothoid_arcfunction4(anochpoint_x,anochpoint_y,l_11,A1,Curv_In);
			//anochangpoint.y =anochangpoint.y*(-1);

		
			float G_anchpoint_x;
			float G_anchpoint_y;

			G_anchpoint_x =  sP_X + anochpoint_x*cos(sP_Th)- anochpoint_y*sin(sP_Th);
			G_anchpoint_y =  sP_Y + anochpoint_x*sin(sP_Th)+ anochpoint_y*cos(sP_Th);

			float G_Dx =  G_chpoint_x-G_anchpoint_x;
			float G_Dy =  G_chpoint_y-G_anchpoint_y;

			//std::cout<<"G_Dy  G  " <<G_Dy << " "<<G_Dx<<std::endl;

			chpoint_Th = atan2(G_Dy,G_Dx);

			chpoint_Th = chpoint_Th - sP_Th;

			//std::cout<<"G_changepoint.x "<<G_changepoint.x<<" "<<G_changepoint.y<<" "<<changepoint.angle<<std::endl;
			//std::cout<<G_anochangpoint.x<<"  "<<G_anochangpoint.y<<std::endl;
		}
			
		if(Curv_In<0)     //------ - ------ 0ÇúÏß
		{
				clothoid_arcfunction3(chpoint_x,chpoint_y,l_1,A1,Curv_In);
				// changepoint.y =changepoint.y*(-1);		
				float l_11 =  l_1 - 0.1;

	
				float G_chpoint_x;
				float G_chpoint_y;

				G_chpoint_x = sP_X + chpoint_x*cos(sP_Th)-chpoint_y*sin(sP_Th);
				G_chpoint_y = sP_Y + chpoint_x*sin(sP_Th)+chpoint_y*cos(sP_Th);

			

				float  anochpoint_x;
				float  anochpoint_y;
				clothoid_arcfunction3(anochpoint_x,anochpoint_y,l_11,A1,Curv_In);
				//anochangpoint.y =anochangpoint.y*(-1);

				float G_anochpoint_x;
				float G_anochpoint_y;

				G_anochpoint_x =  sP_X + anochpoint_x*cos(sP_Th)-anochpoint_y*sin(sP_Th);
				G_anochpoint_y =  sP_Y + anochpoint_x*sin(sP_Th)+anochpoint_y*cos(sP_Th);

				chpoint_Th= atan2((G_chpoint_y-G_anochpoint_y),(G_chpoint_x-G_anochpoint_x));

				chpoint_Th = chpoint_Th - sP_Th;

				//std::cout<<"G_changepoint "<<G_changepoint.x<<" "<<G_changepoint.y<<" "<<changepoint.angle<<std::endl;
				//std::cout<<G_anochangpoint.x<<"  "<<G_anochangpoint.y<<std::endl;
		}

		if(nowCurv*DetaCurv<=0)       //ÎŽµœÇúÂÊÎª0µÄµãÖ®Ç°µÄÇúÏß
		{
			if(Curv_In>0)       //----+  ---- 0ÇúÏß
			{
				clothoid_arcfunction4(X,Y,l,A1,Curv_In);
				//Y =Y *-1;			
			}
			if(Curv_In<0)     //------ - ------ 0ÇúÏß
			{
				clothoid_arcfunction3(X,Y,l,A1,Curv_In);
				// Y =Y *-1 ;
			}
		}
		else              //µœ0Ö®ºóµÄÇúÏß²¿·Ö
		{
			float x,y;
			if(Fial_Curv>0)
			{
				float l2 = l - l_1;
				clothoid_arcfunction3(x,y,l2,A2,0);
				//y = y*(-1);
	
				X = chpoint_x + x*cos(chpoint_Th)-y*sin(chpoint_Th);
				Y = chpoint_y + x*sin(chpoint_Th)+y*cos(chpoint_Th);
			}

			if(Fial_Curv<0)
			{
				float l2 = l - l_1;
				clothoid_arcfunction4(x,y,l2,A2,0);
				//clothoid_arcfunction3(x,y,l2,A2,0);
				//y = y*(-1);

				X = chpoint_x + x*cos(chpoint_Th)-y*sin(chpoint_Th);
				Y = chpoint_y + x*sin(chpoint_Th)+y*cos(chpoint_Th);
			}
		}
}

void Clothoid_arcfinder::computeLine(float &X,float &Y,const float l)
{
	Y = 0;
	X = l;
}

void Clothoid_arcfinder::computeArc(float &X,float &Y,const float l,const float &Curv_In)
{
	float R;
	R = 1/Curv_In;
	R = sqrt(R*R);
	float th;
	th = l/R;
	if(Curv_In>0)  ///Î»ÓÚžÃµãœšÁ¢×ø±êÏµµÄÕýY°ëÖá
	{
		float CircleX,CircleY;
		CircleX = 0;
		CircleY = R;
		X = CircleX + R*cos(th - 3.14/2);
		Y = CircleY + R*sin(th - 3.14/2);
	}
	if(Curv_In<0)
	{
		float CircleX,CircleY;
		CircleX = 0;
		CircleY = -R;
		X = CircleX + R*cos(3.14/2 - th);
		Y = CircleY + R*sin(3.14/2 - th);
	}
}

void Clothoid_arcfinder::clothoid_arcP_XY(float &EX,float &EY,float &ECurv,const float &sP_X,const float &sP_Y,const float &sP_Th,const float &Curv_In,const float &DetaCurv, float &Len)
{
	float A;
	//float DetaCurv;
	
	float l = Len;
	
	const float L=samplelength;
	
	float Fial_Curv = Curv_In + samplelength * DetaCurv;
	
	//float L_step =0.1;
	A = L/(Fial_Curv-Curv_In);
	A = sqrt(A*A);
	float mixFialIn_Curv;
	mixFialIn_Curv = Fial_Curv*Curv_In;
	
	int mode;
	if(DetaCurv>0)
	{
		if(mixFialIn_Curv>=0)                //ÔöŽó£¬Í¬·ûºÅÏÂ
		{
			if(Fial_Curv>0 || Curv_In>0)//+*+ÔöŽó
			{
				mode = 11;
			}
			if(Fial_Curv<0 || Curv_In<0)   //-*- ÔöŽó
			{
				mode = 12;
			}
		}
		else    //ÔöŽó£¬ÒìºÅ  - ---- +
		{
				mode = 13;
		}
	}
	
	if(DetaCurv ==0)
	{
			if(Curv_In==0)//ÇúÂÊ²»±ä»¯£¬ÇÒÎª0£¬Ö±Ïß
			{
				mode = 21;
			}
			else//Ò»¶ÎÔ²»¡£¬ÇúÂÊÎªCurv_In
			{
				mode = 22;
			}
	}
	
	if(DetaCurv<0)   //ÇúÂÊŒõÐ¡
	{
			if(mixFialIn_Curv>=0)                //ŒõÐ¡£¬Í¬·ûºÅÏÂ
			{
						if(Fial_Curv>0 || Curv_In>0)//+*+ ŒõÐ¡
						{
							mode = 31;
						}
						if(Fial_Curv<0 || Curv_In<0)   //-*- ŒõÐ¡
						{
							mode = 32;
						}
			}
			else    //ŒõÐ¡£¬ÒìºÅ  +---- -
			{
				mode = 33;
			}
	}
	
	float X,Y;

	switch(mode)
	{
		case 11:
			{
				clothoid_arcfunction3(X,Y,l,A,Curv_In);
				 break;
			}
		case 12:
			{
				float C_Curv_In;
				C_Curv_In = sqrt(Curv_In*Curv_In);
				clothoid_arcfunction4(X,Y,l,A,C_Curv_In);
				 Y =Y *-1 ;
				 break;
			}
		case 13:
			{
				connectclothoid_arcP_XY(X,Y,sP_X,sP_Y,sP_Th,l,Curv_In,Fial_Curv,samplelength);
				break;
			}
		case 21:
			{
				computeLine(X,Y,l);
				break;
			}
		case 22:
			{
				computeArc(X,Y,l,Curv_In);
				break;
			}
		case 31:
			{
				clothoid_arcfunction4(X,Y,l,A,Curv_In);
				 //Y =Y *-1;
				break;
			}
		case 32:
			{
				float C_Curv_In;
				C_Curv_In = sqrt(Curv_In*Curv_In);
				clothoid_arcfunction3(X,Y,l,A,C_Curv_In);
				Y =Y *-1 ;
				break;
			}
		case 33:
			{
				connectclothoid_arcP_XY(X,Y,sP_X,sP_Y,sP_Th,l,Curv_In,Fial_Curv,samplelength);
				break;
			}
	}

	EX = sP_X + X*cos(sP_Th)-Y*sin(sP_Th);
	EY = sP_Y + X*sin(sP_Th)+Y*cos(sP_Th);
	ECurv = Curv_In + Len*DetaCurv;
	
	//std::cout<<"EX EY ECurv "<<EX<<" "<<EY<<" "<<ECurv<<std::endl;
}

void Clothoid_arcfinder::Gen_clothoid_arcP_XY(float &EX,float &EY,float &ECurv,float &Clothoid_Len,const float &sP_X,const float &sP_Y,const float &sP_Th,const float &Curv_In,const float &DetaCurv, float &Len)
{
	float frontl = 0.0;
	float rearl = 0.0;
	float l =Len;
	
	float nowCruv = Curv_In + Len*DetaCurv;
	
	if((Curv_In>=Max_Cruv && nowCruv > Max_Cruv) || (Curv_In<=Min_Cruv && nowCruv < Min_Cruv))
	{
		Clothoid_Len = Len;
		float RealDetaCurv = 0.0;
        clothoid_arcP_XY(EX,EY,ECurv,sP_X,sP_Y,sP_Th,Curv_In,RealDetaCurv,l);
		ECurv = Curv_In;
		
	}
	else
	{
	
		if(nowCruv > Max_Cruv || nowCruv < Min_Cruv)
		{
			if(nowCruv>0)
			{
				frontl = (Max_Cruv - Curv_In)/DetaCurv;
				rearl = l - frontl;
				
				Clothoid_Len = frontl;
			
				float X,Y,Cruv,TH;
				clothoid_arcP_XY(X,Y,Cruv,sP_X,sP_Y,sP_Th,Curv_In,DetaCurv,frontl);
				
				float frontl1 = frontl - 0.1;
				float X1,Y1,Cruv1;
				clothoid_arcP_XY(X1,Y1,Cruv1,sP_X,sP_Y,sP_Th,Curv_In,DetaCurv,frontl1);
				TH = atan2((Y - Y1),(X-X1));
				
				float RealDetaCurv = 0.0;
				clothoid_arcP_XY(EX,EY,ECurv,X,Y,TH,Max_Cruv,RealDetaCurv,rearl);
				ECurv = Max_Cruv;
			}
			else
			{
				frontl = (Min_Cruv - Curv_In)/DetaCurv;
				rearl = l - frontl;
				
				Clothoid_Len = frontl;
			
				float X,Y,Cruv,TH;
				clothoid_arcP_XY(X,Y,Cruv,sP_X,sP_Y,sP_Th,Curv_In,DetaCurv,frontl);
				
				float frontl2 = frontl - 0.1;
				float X1,Y1,Cruv1;
				clothoid_arcP_XY(X1,Y1,Cruv1,sP_X,sP_Y,sP_Th,Curv_In,DetaCurv,frontl2);
				TH = atan2((Y - Y1),(X-X1));
				
				float RealDetaCurv = 0.0;
				clothoid_arcP_XY(EX,EY,ECurv,X,Y,TH,Min_Cruv,RealDetaCurv,rearl);
				ECurv = Min_Cruv;
			}
		}
		else
		{
			clothoid_arcP_XY(EX,EY,ECurv,sP_X,sP_Y,sP_Th,Curv_In,DetaCurv,Len);
			Clothoid_Len = Len;
		}
	}
}

float Clothoid_arcfinder::getNodeTh(const Node* Succ,const Node* nPred,const float &DetaCurv,const float &l)
{
	float Th;
	float  l1 = l-0.1;
	Node add_point;
	Gen_clothoid_arcP_XY(add_point.x,add_point.y,add_point.Cruv,add_point.Clothoid_Len,nPred->x,nPred->y,nPred->Th,nPred->Cruv,DetaCurv,l1);
	Th = atan2((Succ->y-add_point.y),(Succ->x-add_point.x));
	return Th;
}

bool Clothoid_arcfinder::existObs(const geometry_msgs::Point &CenterPoint)
{
	float ThreadR = Carwidth/2;

	// std::cout<<"CObssize "<<CObssize<<std::endl;
	
	// std::cout<<CenterPoint.x<<" "<<CenterPoint.y<<std::endl;
	// std::cout<<Map_origin.x<<" "<<Map_origin.y<<" "<<Map_Resolution<<std::endl;




	int Cnx = floor((CenterPoint.x - Map_origin.x)/Map_Resolution);
	int Cny = floor((CenterPoint.y - Map_origin.y)/Map_Resolution);

	// std::cout<<"center index "<<((Cnx)+(Cny)*Map_With)<<std::endl;
	// std::cout<<Cnx<<" "<<Cny<<std::endl;


	//std::cout<<"ThreadR "<<ThreadR<<std::endl;
	
	for(int i = (-CObssize);i<=(CObssize);i++)
	{
		for(int j = (-CObssize);j<=(CObssize);j++)
		{
			float judegeObdis = (i*i+j*j)*(Map_Resolution * Map_Resolution);

			//std::cout<<"judegeObdis "<<judegeObdis<<std::endl;
			
			judegeObdis = sqrt(judegeObdis);

			//std::cout<<"judegeObdis---- "<<judegeObdis<<std::endl;
			
			if(judegeObdis<=ThreadR)
			{
				int index = (Cnx+j)+(Cny+i)*Map_With;
				//std::cout<<"checkindex "<<index<<" ";
				if(index<0 || index>=ObstacleIdx.size())
				{
					continue;
				}
				if(ObstacleIdx[index] == 1)
				{
					return true;
				}
			}
		}
	}
	// std::cout<<std::endl;
	return false;
}


bool Clothoid_arcfinder::checkifObs(const Node*newNode,const float &DetaCurv,const float &l)
{
	Node *fatherNode = newNode->parent;
	float partCarL = CarLength/4;
	
	// std::cout<<"debug checkifObs ------>>>>>>>>>> "<<std::endl;
	// std::cout<<"fatherNode------ "<<fatherNode->x<<" "<<fatherNode->y<<" "<<fatherNode->Th<<std::endl;
	// std::cout<<"newNode------ "<<newNode->x<<" "<<newNode->y<<" "<<newNode->Th<<std::endl;
	// std::cout<<"partCarL " <<partCarL<<std::endl;
	// std::cout<<"final_Curv " <<final_Curv<<std::endl;
	// std::cout<<"l " <<l<<std::endl;
	
	for(float dl=0;dl<=l;dl=dl+1.0)
	{
		//std::cout<<"----- dl ------ "<<dl<<std::endl;
		Node tempNode;
		if(dl==0 || (dl-l)*(dl-l)<0.0001)
		{
			if(dl==0)
			{
			   tempNode = *fatherNode;
			}
			else
			{
			   tempNode = *newNode;
			
			}
		}
		else
		{
		   Gen_clothoid_arcP_XY(tempNode.x,tempNode.y,tempNode.Cruv,tempNode.Clothoid_Len,fatherNode->x,fatherNode->y,fatherNode->Th,fatherNode->Cruv,DetaCurv,dl);
		   tempNode.Th = getNodeTh(&tempNode,fatherNode,DetaCurv,dl);
	       tempNode.parent = fatherNode;
		}
		
		// std::cout<<"tempNode ----->>>>>> "<<tempNode.x<<" "<<tempNode.y<<" "<<tempNode.Th<<" "<<tempNode.Cruv<<std::endl;
		


		geometry_msgs::Point chechpoint;
		chechpoint.x = tempNode.x;
		chechpoint.y = tempNode.y;
		chechpoint.z = 0.0;

		if(existObs(chechpoint))
		{
			  std::cout<<" Ob Ob Ob Ob "<<std::endl;
			  return true;
		}

		// float detaL[2] = {-0.4,0.4};
		// for(int i=0;i<2;i++)
		// {
		// 	float distoC = detaL[i];
		// 	// std::cout<<" distoC "<<distoC<<std::endl;

		// 	geometry_msgs::Point tempoint;
		// 	tempoint.x = tempNode.x + cos(tempNode.Th)*distoC;
		// 	tempoint.y = tempNode.y + sin(tempNode.Th)*distoC;
		// 	tempoint.z = 0;

		// 	// std::cout<<"tempoint.x ----- "<<tempoint.x<<" "<<tempoint.y<<std::endl;
		
		// 	if(existObs(tempoint))
		// 	{
		// 	  std::cout<<" Ob Ob Ob Ob "<<std::endl;
		// 	  return true;
		// 	}
		// }

		// for(float distoC=-partCarL;distoC<=partCarL;distoC=distoC+partCarL)
		// {
		// 	std::cout<<" distoC "<<distoC<<std::endl;
			
		// 	geometry_msgs::Point tempoint;
		// 	tempoint.x = tempNode.x + cos(tempNode.Th)*distoC;
		// 	tempoint.y = tempNode.y + sin(tempNode.Th)*distoC;
		// 	tempoint.z = 0;
			
		// 	std::cout<<"tempoint.x ----- "<<tempoint.x<<" "<<tempoint.y<<std::endl;
				
			
		// 	if(existObs(tempoint))
		// 	{
		// 	  std::cout<<" Ob Ob Ob Ob "<<std::endl;
		// 	  return true;
		// 	}
		// }
	}
	return false;
}

static int count = 0;


Node* Clothoid_arcfinder::SearchAlgorithm(Node* Nodeset)
{
	bool firsttime = true;

	float Mapmaxx = Map_origin.x + Map_With * Map_Resolution;
	float Mapmaxy = Map_origin.y + Map_Height * Map_Resolution;
	std::cout<<"Map_origin.x "<<Map_origin.x<<" "<<Map_origin.y<<std::endl;
	std::cout<<"Mapmaxx.x "<<Mapmaxx<<" "<<Mapmaxy<<std::endl;


	poses3D.poses.clear();
	//std::cout<<"-------------"<<std::endl;
	typedef boost::heap::binomial_heap<Node*, boost::heap::compare<CompareNodes>> priorityQueue;
	priorityQueue O;

	typedef boost::heap::binomial_heap<Node*, boost::heap::compare<CompareNodes>> priorityQueue;
	priorityQueue O1;
	//std::cout<<"============"<<std::endl;
	
	int iPred,igoal,iSucc;
	
	//float NewG;
	
	updateH(&StartNode);
	
	//std::cout<<"Node StartNode ---->> " <<StartNode.x<<" "<<StartNode.y<<" "<<StartNode.Th<<" "<<StartNode.Cruv<<" "<<StartNode.G<<" "<<StartNode.H<<" "<<StartNode.parent<<" "<<StartNode.ol<<" "<<StartNode.cl<<" "<<StartNode.obd<<std::endl;
   //std::cout<<"Node GoalNode ---->> " << GoalNode.x<<" "<<GoalNode.y<<" "<<GoalNode.Th<<" "<<GoalNode.Cruv<<" "<<GoalNode.G<<" "<<GoalNode.H<<" "<<GoalNode.parent<<" "<<GoalNode.ol<<" "<<GoalNode.cl<<" "<<GoalNode.obd<<std::endl;
	
	iPred = setIdex(&StartNode);
	
	igoal = setIdex(&GoalNode);
	
   	//std::cout<<"------ iPred ------ "<<iPred<<std::endl;
   	//std::cout<<"------ igoal ------ "<<igoal<<std::endl;
   
   if(Nodeset[iPred].obd == 1 || Nodeset[igoal].obd == 1)
   {
		std::cout<<"source or goal point inobstacle "<<std::endl;
	    return nullptr;
   }
   
   StartNode.ol = true;
   StartNode.cl = false;
   Nodeset[iPred] = StartNode;
   
   //std::cout<<"after-------->>>>>>>>>>>>>> "<<StartNode.x<<" "<<StartNode.y<<" "<<StartNode.Th<<" "<<StartNode.Cruv<<" "<<StartNode.G<<" "<<StartNode.H<<" "<<StartNode.parent<<" "<<StartNode.ol<<" "<<StartNode.cl<<" "<<StartNode.obd<<std::endl;
   //std::cout<<"sfter ------->>>>>>>>>>>>>> "<<GoalNode.x<<" "<<GoalNode.y<<" "<<GoalNode.Th<<" "<<GoalNode.Cruv<<" "<<GoalNode.G<<" "<<GoalNode.H<<" "<<GoalNode.parent<<" "<<GoalNode.ol<<" "<<GoalNode.cl<<" "<<GoalNode.obd<<std::endl;
   //std::cout<<"Nodeset[iPred] >>>>>>>>>>>> "<<Nodeset[iPred].x<<" "<<Nodeset[iPred].y<<" "<<Nodeset[iPred].Th<<" "<<Nodeset[iPred].Cruv<<" "<<Nodeset[iPred].G<<" "<<Nodeset[iPred].H<<" "<<Nodeset[iPred].parent<<" "<<Nodeset[iPred].ol<<" "<<Nodeset[iPred].cl<<" "<<Nodeset[iPred].obd<<std::endl; 
   
    O.push(&StartNode);
	
	Node* nPred; //= nullptr;
    Node* nSucc; //= nullptr;
	
	//Node* pre = nullptr;
   //Node* succ = nullptr;
	
   //for (priorityQueue::ordered_iterator it = O.ordered_begin(); it != O.ordered_end(); ++it)
   //{
	   //succ = (*it);
	   //std::cout<<succ->x<<" "<<succ->y<<" "<<succ->Th<<" "<<succ->Cruv<<" "<<succ->G<<" "<<succ->H<<" "<<succ->ol<<" "<<succ->cl<<" "<<succ->obd<<" "<<succ->parent<<std::endl; 
  // }
   
   //delete succ;
   //delete pre;
	
	//std::cout<<"------ Debug ------ ObstacleIdx "<<ObstacleIdx.size()<<std::endl;
	
	while(!O.empty())
	{
		Node* succ = nullptr;
	
		//std::cout<<" -----------	OPEN LIST ---------------- "<<std::endl;
   		//for (priorityQueue::ordered_iterator it = O.ordered_begin(); it != O.ordered_end(); ++it)
   		//{
	   	//	succ = (*it);
	   	//	std::cout<<succ->x<<" "<<succ->y<<" "<<succ->Th<<" "<<succ->Cruvrate<<" "<<succ->Cruv<<" "<<succ->G<<" "<<succ->H<<" "<<succ->pf<<" "<<succ->parent<<std::endl; 
  		//}
   		//delete succ;

   		std::cout<<"O.size() "<<O.size()<<std::endl;
		
		if(O.size()> 5000)
		{
			nPred = O1.top();
			std::cout<<"return node "<<nPred->x<<" "<<nPred->y<<" "<<nPred->parent<<std::endl;
			return nPred;
		}
		
		//std::cout<<"O.size() "<<O.size()<<std::endl;
		
		nPred = O.top();

		if(!firsttime)
		{
			if(nPred->parent != &StartNode)
			{
				O1.push(nPred);
			}
		}


		firsttime = false;
		//std::cout<<"-------"<<nPred->x<<" "<<nPred->y<<" "<<nPred->Cruv<<" "<<std::endl;
		
		iPred = setIdex(nPred);
   
		//std::cout<<"<<<<< iPred >>>>>>> "<<iPred<<std::endl;
		
		if(Nodeset[iPred].cl)
		{
			std::cout<<"in close "<<Nodeset[iPred].x<<" "<<Nodeset[iPred].y<<std::endl;
			O.pop();
			continue;
		}
		else
		{
			std::cout<<"<<<<<<<<<<<<<<<<<< get into search >>>>>>>>>>>>>>>>>>>>>>>>>>> "<<std::endl;
			
			if(Nodeset[iPred].ol)
			{
				Nodeset[iPred].cl = true;
				Nodeset[iPred].ol = false;
				O.pop();
				
				if(Reachgoal(nPred)/*sqrt((nPred->x-GoalNode.x)*(nPred->x-GoalNode.x)+(nPred->y-GoalNode.y)*(nPred->y-GoalNode.y))<=2.0 &&  sqrt((nPred->Th - GoalNode.Th)*(nPred->Th - GoalNode.Th))<=0.349/*(nPred->x == GoalNode.x) && (nPred->y == GoalNode.y) && (nPred->Th == GoalNode.Th)*/)
				{
				  std::cout<<"find path "<<std::endl;
			      return nPred;
				}
				else
				{
					for(float Dc = Curvminrate;Dc<=Curvmaxrate;Dc = Dc + DCCurv)
					{

						
						float final_Curv =  nPred->Cruv + Dc * samplelength;
						
						float l = samplelength;
						
						Node NewNode;
						
						NewNode.Cruvrate = Dc;
						
						Gen_clothoid_arcP_XY(NewNode.x,NewNode.y,NewNode.Cruv,NewNode.Clothoid_Len,nPred->x,nPred->y,nPred->Th,nPred->Cruv,NewNode.Cruvrate,l);
						
						NewNode.Th = getNodeTh(&NewNode,nPred,NewNode.Cruvrate,l);
						
						NewNode.parent =  nPred;
						
						nSucc = &NewNode;
						
						iSucc = setIdex(nSucc);


					   
					  //std::cout<<" parent node "<<std::endl;
					  //std::cout<<nPred->x<<" "<<nPred->y<<" "<<nPred->Th<<" "<<nPred->Cruv<<" "<<final_Curv<<" "<<l<<std::endl;
					  //std::cout<<"final_Curv "<<final_Curv<<std::endl;
					  //std::cout<<"l "<<l<<std::endl;
					  
					   // std::cout<<"deepth "<<deepth<<std::endl;
					   
						
					   if(((nSucc->x)>= Map_origin.x && (nSucc->x)<=Mapmaxx) && ((nSucc->y)>=Map_origin.y && (nSucc->y)<=Mapmaxy) && (!checkifObs(nSucc,nSucc->Cruvrate,l)))
					   {
						   nSucc->obd = -1;

						   // std::cout<<"deepth "<<deepth<<" "<<iSucc<<std::endl;

						   // std::cout<<nSucc->x<<" "<<nSucc->y<<" "<<nSucc->Th<<std::endl;

						  
						  // if(nSucc->y>0)
						  // {
						  // 	std::cout<<"***************** node and parent node *****************************"<<std::endl;
						  //  	std::cout<<" parent node "<<std::endl;
						  //  	std::cout<<nPred->x<<" "<<nPred->y<<" "<<nPred->Th<<" "<<nPred->Cruv<<" "<<nPred->Cruvrate<<" "<<l<<std::endl;
						  //  	std::cout<<"generate node----New Node ------------- "<<std::endl;
					   //     	std::cout<<"generate node>>>>> "<<nSucc->x<<" "<<nSucc->y<<" "<<nSucc->Th<<" "<<nSucc->Cruv<<" "<<nSucc->Cruvrate<<std::endl;
						  //  	std::cout<<"----- count ---- "<<count<<std::endl;
						  //  	std::cout<<"***************** node and parent node *****************************"<<std::endl;
						  // }
						   // std::cout<<"***************** node and parent node *****************************"<<std::endl;
						   // std::cout<<" parent node "<<std::endl;
						   // std::cout<<nPred->x<<" "<<nPred->y<<" "<<nPred->Th<<" "<<nPred->Cruv<<" "<<nPred->Cruvrate<<" "<<l<<std::endl;
						   // std::cout<<"generate node----New Node ------------- "<<std::endl;
					    //    std::cout<<"generate node>>>>> "<<nSucc->x<<" "<<nSucc->y<<" "<<nSucc->Th<<" "<<nSucc->Cruv<<" "<<nSucc->Cruvrate<<std::endl;
						   // std::cout<<"----- count ---- "<<count<<std::endl;
						   // std::cout<<"***************** node and parent node *****************************"<<std::endl;
						   

						   count = count +1;
						   
						   //Visualiziation.publishNode(nSucc);
						   publishNode(nSucc);
						   
						   if(!Nodeset[iSucc].cl)
						   {
							   updateG(nSucc);
							   updateNode_pf(nSucc);
							  //updatepf(nSucc);
							  
							   //std::cout<<"nSucc->G "<<nSucc->G<<std::endl;
							  
							  if(Nodeset[iSucc].ol)
							  {
									updateH(nSucc);
									nSucc->C = nSucc->G + nSucc->H + nSucc->pf;
									//nSucc->C = nSucc->G + nSucc->H;
									
									//std::cout<<"nSucc->G "<<nSucc->G<<" nSucc->H "<<nSucc->H<<" nSucc->C "<<nSucc->C<<std::endl;
									
									if((nSucc->C)<(Nodeset[iSucc].C))
									{
										//std::cout<<"生成节点在openlist C比原来节点的小 "<<std::endl;
										nSucc->cl = false;
										nSucc->ol = true;
										Nodeset[iSucc] = *nSucc;
										O.push(&Nodeset[iSucc]);
									   // delete nSucc;
										nSucc = nullptr;
									}
									else
									{
										//std::cout<<"生成的节点在open里且C比原来节点的大else "<<std::endl;
										//delete nSucc;
										nSucc = nullptr;
									}//生成的节点在open里且C比原来节点的大else
							  }
							  else
							  {
									updateH(nSucc);
									nSucc->C = nSucc->G + nSucc->H + nSucc->pf;
									//nSucc->C = nSucc->G + nSucc->H;
									
									//std::cout<<"nSucc->C "<<nSucc->C<<" nSucc->H "<<nSucc->H<<std::endl;
									
									nSucc->cl = false;
									nSucc->ol =true;
									Nodeset[iSucc] = *nSucc;
									O.push(&Nodeset[iSucc]);

									// std::cout<<"生成的节点既不在close列表里也不在open里  else... "<<std::endl;
									
									//delete nSucc;
									nSucc = nullptr;
									
							  }//生成的节点既不在close列表里也不在open里  else...
							}
							else
							{
								// std::cout<<"生成的节点在close列表里else... "<<std::endl;
								
								//delete nSucc;
								nSucc = nullptr;
							} //生成的节点在close列表里else...
						}
						else
						{

							// std::cout<<"生成的节点位于障碍区域,else..... "<<std::endl;
							// std::cout<<"generate node>>>>> "<<nSucc->x<<" "<<nSucc->y<<" "<<nSucc->Th<<" "<<nSucc->Cruv<<" "<<nSucc->Cruvrate<<std::endl;
							
							//delete nSucc;
							nSucc = nullptr;
						}//生成的节点位于障碍区域,else.....
				   }//for循环
			   }//判断是否到达目标的else......
			}//if(Nodeset[iPred].ol)....
		}//得到的节点不在close列表里,if(Nodeset[iPred].cl) else....
	}//搜索主循环while
	
	if (O.empty()) 
	{
		if(O1.empty())
		{
			std::cout<<"O emptty "<<std::endl;
   	 		return nullptr;
		}
		else
		{
			Node *returnnode;
			returnnode = O1.top();
			std::cout<<"return node "<<returnnode->x<<" "<<returnnode->y<<" "<<returnnode->parent<<std::endl;
			return(returnnode);
		}
	 
  	}
 	return nullptr;				
}

void Clothoid_arcfinder::tracePath(const Node* node)
{
	if(node == nullptr)
	{
		std::cout<<std::endl;
		return;
	}
	
	std::cout<<"final node "<<std::endl;
	std::cout<<node->x<<" --- "<<node->y<<" --- "<<node->Cruv<<" --- "<<node->Clothoid_Len<<" --- "<<node->Cruvrate<<" --- "<<node->Th<<" --- "<<node->parent<<">>>> ";	
	
	final_path.push_back(*node);
	tracePath(node->parent);
}

bool Clothoid_arcfinder::make_plan()
{
	 final_path.clear();
	 Befinal_path.clear();
	 
	 Node* Nodeset = new Node[deepth]();


	 std::cout<<"deepth-------- "<<deepth<<std::endl;
	 
	 setNodeset(Nodeset);
	
	 CObssize = Carwidth/2/Map_Resolution;
	 
	 int startNodeidx = setIdex(&StartNode);
	
	 std::cout<<"startNodeidx>>>>>>> "<<startNodeidx<<std::endl;
	 // std::cout<<"Nodeset[startNodeidx].obd "<<Nodeset[startNodeidx].obd<<std::endl;
	 std::cout<<"StartNode "<<StartNode.x<<" "<<StartNode.y<<" "<<StartNode.Th<<std::endl;	   
	 StartNode.obd = Nodeset[startNodeidx].obd;
	 
	///  //update all map proteinal fields
	//int pfstartindex = setpfindex(&startNode);
	//std::cout<<"pfstartindex "<<pfstartindex<<std::endl;
	// startNode.pf =  Index_APF[pfstartindex];
	 
	 
	//std::cout<<"###################set goal"<<std::endl;

	int goalNodeidx = setIdex(&GoalNode);

	std::cout<<"goalNodeidx >>>>>>>>>>>>> "<<goalNodeidx<<std::endl;
	std::cout<<"goalNode "<<GoalNode.x<<" "<<GoalNode.y<<" "<<GoalNode.Th<<std::endl;	   
	//std::cout<<" Map_With "<< Map_With<<" Map_Height "<<Map_Height<<" Map_origin "<<Map_origin.x<<" "<<Map_origin.y<<std::endl;
		   
	GoalNode.obd = Nodeset[goalNodeidx].obd;
	 


	// std::cout<<"Nodeset[goalNodeidx].obd "<<Nodeset[goalNodeidx].obd<<std::endl;


	std::cout<<" ---- pppppppassss ------ "<<std::endl;
	///update all map proteinal fields
	//int pfgoalindex = setpfindex(&goalNode);
	//std::cout<<"pfgoalindex "<<pfgoalindex<<std::endl;
	//std::cout<<"Index_APF.size() "<<Index_APF.size()<<std::endl;
	
	// goalNode.pf =  Index_APF[pfgoalindex];
	
	//std::cout<<"Node start ---->> " <<startNode.x<<" "<<startNode.y<<" "<<startNode.Th<<" "<<startNode.Cruv<<" "<<startNode.G<<" "<<startNode.H<<" "<<startNode.parent<<" "<<startNode.ol<<" "<<startNode.cl<<" "<<startNode.obd<<std::endl;

    //std::cout<<"Node goal ---->> " << goalNode.x<<" "<<goalNode.y<<" "<<goalNode.Th<<" "<<goalNode.Cruv<<" "<<goalNode.G<<" "<<goalNode.H<<" "<<goalNode.parent<<" "<<goalNode.ol<<" "<<goalNode.cl<<" "<<goalNode.obd<<std::endl;

	clock_t start,finish;
   	double totaltime;
   	start=clock();
	
	Node* result = SearchAlgorithm(Nodeset);
	
	finish=clock();
   	totaltime=(double)(finish-start)/CLOCKS_PER_SEC;
   	std::cout<<"\n此程序的运行时间为"<<totaltime<<"秒！"<<std::endl;
	
	tracePath(result);
	
	if(final_path.empty())
	{
		delete [] Nodeset;
	    	std::cout<<"wrong "<<std::endl;
		return false;
	
	}
	else
	{
		delete [] Nodeset;
		std::cout<<"set OK "<<std::endl;
		

		if(final_path.size()>5)
		{	
			std::cout<<"pit of size "<<std::endl;
			int pathsize = final_path.size();
			for(int j = (pathsize-5);j<pathsize;j++)
			{
				Befinal_path.push_back(final_path[j]);
			}

			final_path.clear();
			final_path = Befinal_path;
		}
  

		std::cout<<"final path is ....... "<<std::endl;
  		for(int u=0;u<final_path.size();u++)
  		{
  			std::cout<<"u "<<u<<std::endl;
  			std::cout<<final_path[u].x<<" "<<final_path[u].y<<" "<<final_path[u].Cruvrate<<" "<<final_path[u].Cruv<<" "<<final_path[u].parent<<" "<<std::endl;
  		}
  		std::cout<<std::endl;

		return true;
			 
		//pubfinalpathNode(final_path);

		//pubfianalpath(final_path);   
	}
}

void Clothoid_arcfinder::SetControlpoints()
{
	controlpoints.clear();
	float Robv = 1.0;
	for(int i =(final_path.size()-1);i>0;i--)
	{
		if(final_path[i-1].Clothoid_Len == samplelength)
		{
			float controltime = final_path[i-1].Clothoid_Len/Robv;
			Executepoint temexecutepoint;
			temexecutepoint.SecondEx = false;
			temexecutepoint.Firstconut = int (controltime/0.1);
			temexecutepoint.Secondcount = -1;
			
			std::cout<<"--------------- "<<std::endl;
			std::cout<<"final_path[i].Cruv "<<final_path[i].Cruv<<std::endl;
			std::cout<<"final_path[i-1].Cruv "<<final_path[i-1].Cruv<<std::endl;
			

			float temradin2 = atan(final_path[i-1].Cruv*1.7);
			float temradin1 = atan(final_path[i].Cruv*1.7);
			temradin2 = toDegree1(temradin2);
			temradin1 = toDegree1(temradin1);
			temexecutepoint.Pos = temradin2;
			temexecutepoint.Rotation = (temradin2 - temradin1)/controltime;
			controlpoints.push_back(temexecutepoint);
		}
		else
		{
			float ALLcontroltime = samplelength/Robv;
			float controltime = final_path[i-1].Clothoid_Len/Robv;
			Executepoint temexecutepoint;
			temexecutepoint.SecondEx = true;
			temexecutepoint.Firstconut = int (controltime/0.1);
			temexecutepoint.Secondcount = (ALLcontroltime - controltime)/0.1;
			
			
			std::cout<<"--------------- "<<std::endl;
			std::cout<<"final_path[i].Cruv "<<final_path[i].Cruv<<std::endl;
			std::cout<<"final_path[i-1].Cruv "<<final_path[i-1].Cruv<<std::endl;
			
			float temradin2 = atan(final_path[i-1].Cruv*1.7);
			float temradin1 = atan(final_path[i].Cruv*1.7);
			temradin2 = toDegree1(temradin2);
			temradin1 = toDegree1(temradin1);
				
			temexecutepoint.Pos = temradin2;
			temexecutepoint.Rotation = (temradin2 - temradin1)/controltime;
			controlpoints.push_back(temexecutepoint);
		}
	}
}


bool Clothoid_arcfinder::ChechReachG(const CPoint2d &Robpose)
{
	float GoalvectX = cos(Goalpoint.pointangle);
	float GoalvectY = sin(Goalpoint.pointangle);
	
	float VerticalGX;
	float VerticalGY;
	
	VerticalGX = -(GoalvectY/sqrt(GoalvectX*GoalvectX+GoalvectY*GoalvectY));
	VerticalGY = (GoalvectX/sqrt(GoalvectX*GoalvectX+GoalvectY*GoalvectY));
	
	
	float toGdisvectX;
	float toGdisvectY;
	
	toGdisvectX = (Robpose.x - Goalpoint.x);
	toGdisvectY = (Robpose.y - Goalpoint.y);
	
	float CostoGth;
	float toGth;
	
	float toGdisvectlen = sqrt(toGdisvectX*toGdisvectX+toGdisvectY*toGdisvectY);

	float VerticalGlen = sqrt(VerticalGX*VerticalGX+VerticalGY*VerticalGY);
	
	CostoGth = (toGdisvectX*VerticalGX + VerticalGY*toGdisvectY)/(toGdisvectlen*VerticalGlen);
	
	
	toGth = acos(CostoGth);
	
	
	float DistoGoal;
	
	DistoGoal = sqrt((Robpose.x - Goalpoint.x)*(Robpose.x - Goalpoint.x)+(Robpose.y - Goalpoint.y)*(Robpose.y - Goalpoint.y));
	
	
	if((DistoGoal*sin(toGth))<=2.0)
	{
		std::cout<<"reach final goal "<<std::endl;
		return true;
	}
	return false;
}

static bool isstart = true;
void Clothoid_arcfinder::Getsubgoal(std::string &frame_id,const CPoint2d &Robpose,CPoint2d &subgoal)
{
	bool updateflag = false;

	std::cout<<"Getsubgoal now rob is "<<Robpose.x
									<<" "<<Robpose.y
									<<" "<<Robpose.pointangle
									<<std::endl;
	std::cout<<"Getsubgoal lsat Rob is "<<lastpose.x
									<<" "<<lastpose.y
									<<" "<<lastpose.pointangle
									<<std::endl;

	CPoint2d ORsubgoal;

	if(isstart)
	{
		lastpose = Robpose;
		ORsubgoal = m_refer_path[refer_pathindex];



		if(frame_id=="/camera_init")
		{
			subgoal = ORsubgoal;
		}

		if(frame_id=="/rslidar")
		{
			//二维坐标系旋转平移
		
			subgoal = tansformWorldtoRob(ORsubgoal,Robpose);
			
			// float dx,dy;
			// dx = ORsubgoal.x - Rob.x;
			// dy = ORsubgoal.y - Rob.y;
			// subgoal.x  = dx * cos(Rob.pointangle) +　dy * sin(Rob.pointangle);
			// subgoal.y = dy  * cos(Rob.pointangle) -　dx * sin(Rob.pointangle);
			// subgoal.pointangle = Rob.pointangle - ORsubgoal.pointangle;
			
			//subgoal.x = ORsubgoal.x - Rob.x;
			//subgoal.y = ORsubgoal.y - Rob.y;
			//subgoal.pointangle = ORsubgoal.pointangle;
			
		}
		
		isstart = false;

		updateflag = true;
		

		/*//way1

		float dx = ORsubgoal.x - Robpose.x;

		float dy = ORsubgoal.y - Robpose.y;

		float vectx = cos(Robpose.pointangle);

		float vecty = sin(Robpose.pointangle);

		float vecfenmu = dx * vectx + dy * vecty;

		float vectvalue = vecfenmu/(sqrt((dx * dx) + (dy * dy)) * 1);

		float th = acos(vectvalue);

		float len = sqrt(dx * dx + dy * dy);

		subgoal.x = len * cos(th);
		
		subgoal.y = len * sin(th);

		subgoal.pointangle = ORsubgoal.pointangle - Robpose.pointangle;*/


		//way2

		// float dx = ORsubgoal.x - Robpose.x;

		// float dy = ORsubgoal.y - Robpose.y;

		// float th = Robpose.pointangle;

		// subgoal.x = dx * cos(th) - dy * sin(th);

		// subgoal.y = dx * sin(th) + dy * cos(th);

		// subgoal.pointangle = ORsubgoal.pointangle - Robpose.pointangle;




		//way3

		// float th = Robpose.pointangle;

		// float R_x = Robpose.x * cos(th) - Robpose.y * sin(th);

		// float R_y = Robpose.x * sin(th) + Robpose.y * cos(th);

		// float O_x = ORsubgoal.x * cos(th) - ORsubgoal.y * sin(th);

		// float O_y = ORsubgoal.x * sin(th) + ORsubgoal.y * cos(th);


		// subgoal.x = O_x - R_x;
		
		// subgoal.y = O_y - R_y;

		// subgoal.pointangle = ORsubgoal.pointangle - Robpose.pointangle;



		// CPoint2d formerpoint;

		// formerpoint.x = ORsubgoal.x + 0.5 * cos(ORsubgoal.pointangle);

		// formerpoint.y = ORsubgoal.y + 0.5 * sin(ORsubgoal.pointangle);


		// std::cout<<"refer_pathindex "<<refer_pathindex<<" "<<m_refer_path[refer_pathindex].x<<" "
		// 		<<m_refer_path[refer_pathindex].y<<" "<<m_refer_path[refer_pathindex].pointangle<<" "
		// 		<<atan2(m_refer_path[refer_pathindex].y,m_refer_path[refer_pathindex].x)<<std::endl;
		// std::cout<<"Robpose "<<Robpose.x<<" "<<Robpose.y<<" "<<Robpose.pointangle<<" "<<atan2(Robpose.y,Robpose.x)<<std::endl;
		// std::cout<<"ORsubgoal "<<ORsubgoal.x<<" "<<ORsubgoal.y<<" "<<ORsubgoal.pointangle<<std::endl;

		// float th = (ORsubgoal.pointangle - Robpose.pointangle);

		// float dx = ORsubgoal.x - Robpose.x;

		// float dy = ORsubgoal.y - Robpose.y;

		// float len = sqrt((dx * dx) + (dy * dy));



		// std::cout<<"dx "<<dx<<" "<<dy<<" "<<th<<std::endl;






		// // float fdx = formerpoint.x - Robpose.x;

		// // float fdy = formerpoint.y - Robpose.y;

		// // float ax = fdx * cos(th) - fdy * sin(th);

		// // float ay = fdx * sin(th) + fdy * cos(th);


		// subgoal.x = len * cos(th);
		
		// subgoal.y = len * sin(th);

		// subgoal.pointangle = th;
		
		



		/*geometry_msgs::PoseStamped globalsubgoal;
		globalsubgoal.header.stamp = ros::Time();
		globalsubgoal.header.frame_id="/camera_init";
		globalsubgoal.pose.position.x = ORsubgoal.x;
		globalsubgoal.pose.position.y = ORsubgoal.y;
		globalsubgoal.pose.position.z = 0.0;

		tf::TransformListener listener_;
	    tf::Transform transform;
	    tf::Quaternion q;
	    q.setRPY(0, 0, ORsubgoal.pointangle);
    	transform.setRotation(q);


	    globalsubgoal.pose.orientation.x = transform.getRotation().getX();
	    globalsubgoal.pose.orientation.y = transform.getRotation().getY();
	    globalsubgoal.pose.orientation.z = transform.getRotation().getZ();
	    globalsubgoal.pose.orientation.w = transform.getRotation().getW();


	    tf::Stamped<tf::Pose> goal_pose, global_pose;
    	poseStampedMsgToTF(globalsubgoal, goal_pose);

    //just get the latest available transform... for accuracy they should send
    //goals in the frame of the planner
   		goal_pose.stamp_ = ros::Time();

    	try{
    		listener_.waitForTransform("/rslidar","/camera_init",ros::Time(0),ros::Duration(1.0));
      		listener_.transformPose("/rslidar",ros::Time(0),goal_pose,"/camera_init",global_pose);
    	}
   		catch(tf::TransformException& ex){
     					ROS_ERROR("%s",ex.what());
	          			ros::Duration(1.0).sleep();
   		}

	    geometry_msgs::PoseStamped nsubgoal;
	    tf::poseStampedTFToMsg(global_pose, nsubgoal);

	    double x,y,z,w;				//姿态四元数
		x = nsubgoal.pose.orientation.x;
		y = nsubgoal.pose.orientation.y;
		z = nsubgoal.pose.orientation.z;
		w = nsubgoal.pose.orientation.w;
		double yaw, pitch, roll;
		tf::Matrix3x3(tf::Quaternion(x, y, z, w)).getRPY(roll, pitch, yaw);
		    
		subgoal.x = nsubgoal.pose.position.x;
		subgoal.y = nsubgoal.pose.position.y;
		subgoal.pointangle = yaw;
*/





		/*geometry_msgs::PoseStamped globalsubgoal;
		globalsubgoal.header.stamp=ros::Time::now();
		globalsubgoal.header.frame_id="/camera_init";
		globalsubgoal.pose.position.x = ORsubgoal.x;
		globalsubgoal.pose.position.y = ORsubgoal.y;
		globalsubgoal.pose.position.z = 0.0;
		
		tf::TransformListener listener_;
		tf::Transform transform;
	    tf::Quaternion q;
	    q.setRPY(0, 0, ORsubgoal.pointangle);
	    //tf::Transform transform;
	    // transform.setRotation(q);
    	transform.setRotation(q);
    	// transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));

        // globalsubgoal.pose.orientation = transform.getRotation();


	    globalsubgoal.pose.orientation.x = transform.getRotation().getX();
	    globalsubgoal.pose.orientation.y = transform.getRotation().getY();
	    globalsubgoal.pose.orientation.z = transform.getRotation().getZ();
	    globalsubgoal.pose.orientation.w = transform.getRotation().getW();

	    geometry_msgs::PoseStamped nsubgoal;
	    try{
	            listener_.transformPose("/rslidar",globalsubgoal,nsubgoal);
	       }
	    catch (tf::TransformException &ex) {
	          ROS_ERROR("%s",ex.what());
	          ros::Duration(1.0).sleep();
	    }

	    double x,y,z,w;				//姿态四元数
	    x = nsubgoal.pose.orientation.x;
	    y = nsubgoal.pose.orientation.y;
	    z = nsubgoal.pose.orientation.z;
	    w = nsubgoal.pose.orientation.w;
		double yaw, pitch, roll;
	    tf::Matrix3x3(tf::Quaternion(x, y, z, w)).getRPY(roll, pitch, yaw);
	    
		subgoal.x = nsubgoal.pose.position.x;
		subgoal.y = nsubgoal.pose.position.y;
		subgoal.pointangle = yaw;*/

		// float th = Robpose.pointangle;
		// float dx = ORsubgoal.x - Robpose.x;
		// float dy = ORsubgoal.y - Robpose.y;
		// float dth = ORsubgoal.pointangle - Robpose.pointangle;

		// //xuanzhuanpingyizhidituzuobiaoxi
		// subgoal.x = dx * cos(th) + dy * sin(th);
		// subgoal.y = dy*cos(th) - dx * sin(th);
		// subgoal.pointangle = th;


		// subgoal.x = ORsubgoal.x - Robpose.x;
		// subgoal.y = ORsubgoal.y - Robpose.y;
		// subgoal.pointangle = ORsubgoal.pointangle;
		
	}
	
	if(m_refer_path.size()<refer_pathindex)
	{
		refer_pathindex = m_refer_path.size() - 1;
		ORsubgoal = m_refer_path[refer_pathindex];


		if(frame_id=="/camera_init")
		{
			subgoal = ORsubgoal;
		}

		if(frame_id=="/rslidar")
		{
			//二维坐标系旋转平移
			//subgoal.x = ORsubgoal.x - Rob.x;
			//subgoal.y = ORsubgoal.y - Rob.y;
			// subgoal.pointangle = ORsubgoal.pointangle;
			
			// float dx,dy;
			
			// dx = ORsubgoal.x - Rob.x;
			// dy = ORsubgoal.y - Rob.y;
			
			// subgoal.x  = dx * cos(Rob.pointangle) +　dy * sin(Rob.pointangle);
			// subgoal.y = dy  * cos(Rob.pointangle) -　dx * sin(Rob.pointangle);
			// subgoal.pointangle = Rob.pointangle - ORsubgoal.pointangle;
		
			subgoal = tansformWorldtoRob(ORsubgoal,Robpose);
		}

		updateflag = true;
	}
	
	if(((Robpose.x - lastpose.x) * (Robpose.x - lastpose.x) + (Robpose.y - lastpose.y) * (Robpose.y - lastpose.y))>1.0)
	{
		std::cout<<"get in ^^^^^^^ "<<std::endl;
		lastpose = Robpose;
		refer_pathindex = refer_pathindex + 1;
		if(refer_pathindex==(m_refer_path.size()-1))
		{
			refer_pathindex = m_refer_path.size() - 1;
		}

		ORsubgoal = m_refer_path[refer_pathindex];

		/*std::cout<<"refer_pathindex "<<refer_pathindex<<" "<<m_refer_path[refer_pathindex].x<<" "
				<<m_refer_path[refer_pathindex].y<<" "<<m_refer_path[refer_pathindex].pointangle<<" "
				<<atan2(m_refer_path[refer_pathindex].y,m_refer_path[refer_pathindex].x)<<std::endl;
		std::cout<<"Robpose "<<Robpose.x<<" "<<Robpose.y<<" "<<Robpose.pointangle<<" "<<atan2(Robpose.y,Robpose.x)<<std::endl;
		std::cout<<"ORsubgoal "<<ORsubgoal.x<<" "<<ORsubgoal.y<<" "<<ORsubgoal.pointangle<<std::endl;*/


		if(frame_id=="/camera_init")
		{
			subgoal = ORsubgoal;
		}

		if(frame_id=="/rslidar")
		{
			//二维坐标系旋转平移
			// subgoal.x = ORsubgoal.x - Rob.x;
			// subgoal.y = ORsubgoal.y - Rob.y;
			// subgoal.pointangle = ORsubgoal.pointangle;
			
			// float dx,dy;
			
			// dx = ORsubgoal.x - Rob.x;
			// dy = ORsubgoal.y - Rob.y;
			
			// subgoal.x  = dx * cos(Rob.pointangle) +　dy * sin(Rob.pointangle);
			// subgoal.y = dy  * cos(Rob.pointangle) -　dx * sin(Rob.pointangle);
			// subgoal.pointangle = Rob.pointangle - ORsubgoal.pointangle;
			
			subgoal = tansformWorldtoRob(ORsubgoal,Robpose);
		}
	
		updateflag = true;
	}



	if(!updateflag)
	{
		ORsubgoal = m_refer_path[refer_pathindex];
		
		if(frame_id=="/camera_init")
		{
			subgoal = ORsubgoal;
		}
		
		if(frame_id=="/rslidar")
		{
			subgoal = tansformWorldtoRob(ORsubgoal,Robpose);
		}

	}

	std::cout<<"refer_pathindex "<<refer_pathindex<<std::endl;

	std::cout<<"ORsubgoal "<<ORsubgoal.x
									<<" "<<ORsubgoal.y
									<<" "<<ORsubgoal.pointangle
									<<std::endl;
	

	std::cout<<"subgoal.x"<<subgoal.x<<" "<<subgoal.y<<" "<<subgoal.pointangle<<std::endl;
	
	/*subGoal = subgoal;*/
	
}




