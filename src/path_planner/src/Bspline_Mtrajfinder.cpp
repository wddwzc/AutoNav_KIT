#include "path_planner/Bspline_Mtrajfinder.h"
#include "path_planner/smooth.h"
#include "path_planner/computetool.h"
#include "path_planner/Bspline.h"	



Bspline_Mtrajfinder::Bspline_Mtrajfinder()
{
	//路径平滑方法选择标志位
	flagbsplinesmooth = true;
	flagbazersmooth = false; 
	
	
	//初始化机器人位置
	Rob.x = 0.0;
	Rob.y = 0.0;
	Rob.pointangle = 0.0;
	//初始化机器人的上一次位置，
	//记录机器人上一次位置主要为了更新子目标点使用
	lastRob.x = 0.0;
	lastRob.y = 0.0;
	lastRob.pointangle = 0.0;
	
	codingstart = true; //在取目标点时确定是不是程序启动第一次标志位
	//如果是第一次，机器人的上一次位置和当前位置相等
	
	refer_pathindex = 6;//程序启动时取先验路径上索引为6的点作为第一个
						//子目标点，之后由这个点进行更新
	
	Ob_thread = 0.01; //程序中判断障碍距离阈值，0距离的平方值
	sampnum = 200; //对得到的轨迹平滑处理之后采多少个点作为轨迹点
	angletoEnd = 0.0; //每一条轨迹的方向，用作方向向量做一些计算
	
	Strategy_level = 2; //控制导航策略，1为靠右 2为沿中线
	intervaltrajNum = 8; //规定多少条轨迹为一组优先级，
						//然后依次按照这组内的轨迹进行选择
	centerlineoffset = 0.0; //表示按照先验路径走的时候
							//需要先验路径靠右多少为最优的
	
	unitShiftNum = 3; //在可行轨迹都编码到了最右距离之后
					  //控制从距离最右边距离多少出开始为最优轨迹选择的地方
	//定义道路宽度、额外扩展道路宽度
	// m_roadWidth = 8.0; //定义了道路宽度
	// roadwithnum = 4; //通过该变量确定额外需要考虑多少距离内采样轨迹
	// roadways = 32; //通过这个变量和路况得到机器人生成轨迹之前间隔多少距离
	// extra_roadWidth= m_roadWidth/roadwithnum;
	

	// 定义道路宽度、额外扩展道路宽度
	m_roadWidth = 5.0; //定义了道路宽度
	// roadwithnum = 5; //通过该变量确定额外需要考虑多少距离内采样轨迹
	roadways = 20; //通过这个变量和路况得到机器人生成轨迹之前间隔多少距离
	extra_roadWidth = 0;



	judgeOblen = 5.0; //判断轨迹是否接触障碍物时取轨迹的长度
	distoBob = 0.8; //选取轨迹上多少距离处的点作为速度输出的控制点

	show_log = false;
	
}

Bspline_Mtrajfinder::Bspline_Mtrajfinder(std::vector<CPoint2d> key_point)
{
	//路径平滑方法选择标志位
	flagbsplinesmooth = true;
	flagbazersmooth = false;
	
	//初始化机器人位置
	Rob.x = 0.0;
	Rob.y = 0.0;
	Rob.pointangle = 0.0;
	//初始化机器人的上一次位置，
	//记录机器人上一次位置主要为了更新子目标点使用
	lastRob.x = 0.0;
	lastRob.y = 0.0;
	lastRob.pointangle = 0.0;
	
	codingstart = true; //在取目标点时确定是不是程序启动第一次标志位
	//如果是第一次，机器人的上一次位置和当前位置相等
	
	refer_pathindex = 6;//程序启动时取先验路径上索引为6的点作为第一个
						//子目标点，之后由这个点进行更新
	
	Ob_thread = 0.16; //程序中判断障碍距离阈值，0距离的平方值
	sampnum = 200; //对得到的轨迹平滑处理之后采多少个点作为轨迹点
	angletoEnd = 0.0; //每一条轨迹的方向，用作方向向量做一些计算
	
	Strategy_level = 2; //控制导航策略，1为靠右 2为沿中线
	intervaltrajNum = 8; //规定多少条轨迹为一组优先级，
						//然后依次按照这组内的轨迹进行选择
	centerlineoffset = 0.0; //表示按照先验路径走的时候
							//需要先验路径靠右多少为最优的
	
	unitShiftNum = 3; //在可行轨迹都编码到了最右距离之后
					  //控制从距离最右边距离多少出开始为最优轨迹选择的地方
	// //定义道路宽度、额外扩展道路宽度
	// m_roadWidth = 8.0; //定义了道路宽度
	// roadwithnum = 4; //通过该变量确定额外需要考虑多少距离内采样轨迹
	// roadways = 32; //通过这个变量和路况得到机器人生成轨迹之前间隔多少距离
	// extra_roadWidth= m_roadWidth/roadwithnum;

	// 定义道路宽度、额外扩展道路宽度
	m_roadWidth = 5.0; //定义了道路宽度
	// roadwithnum = 5; //通过该变量确定额外需要考虑多少距离内采样轨迹
	roadways = 20; //通过这个变量和路况得到机器人生成轨迹之前间隔多少距离
	extra_roadWidth = 0;
	
	judgeOblen = 5.0; //判断轨迹是否接触障碍物时取轨迹的长度
	distoBob = 0.8; //选取轨迹上多少距离处的点作为速度输出的控制点

	show_log = false;
	
	set_navpoint(key_point);
}

Bspline_Mtrajfinder::~Bspline_Mtrajfinder()
{
	
}

void Bspline_Mtrajfinder::set_navpoint(std::vector<CPoint2d> key_point)
{

	/*std::ofstream writefile;
	writefile.open("/home/robot/map.txt",std::ios::out|std::ios::trunc);

	std::ofstream writefile1;
	writefile1.open("/home/robot/map1.txt",std::ios::out|std::ios::trunc);*/


	if(!m_refer_path.empty())
	{
		m_refer_path.clear();
	}

	//////////////////////////////////////
	// std::cout<<"Clothoid_arcfinder "<<std::endl;
	
/*	std::vector<CPoint2d> allpath;
	for(int u=0;u<key_point.size();u++)
	{
		// std::cout<<"key_point "<<key_point[u].x<<" "<<key_point[u].y<<std::endl;
		writefile<<key_point[u].x<<" "<<key_point[u].y<<std::endl;
		allpath.push_back(key_point[u]);
	}*/



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



	//B样条路径平滑

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
	
	//计算路径角度朝向

	smoothpath = LBSpline.BSplinePoints;

	for(int i=1;i<smoothpath.size();i++)
	{
		float Th = atan2((smoothpath[i].y-smoothpath[i-1].y),(smoothpath[i].x-smoothpath[i-1].x));
		smoothpath[i-1].pointangle = Th;
	}

	//把平滑路径上的点截取为1m一个
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

			//if(dis>=0.2)
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

	for(int i=0;i<m_refer_path.size()-1;i++)
	{
		m_refer_path[i].pointangle = atan2((m_refer_path[i+1].y - m_refer_path[i].y),(m_refer_path[i+1].x - m_refer_path[i].x));
	}

	m_refer_path[m_refer_path.size()-1].pointangle = m_refer_path[m_refer_path.size()-2].pointangle;
	
	Goalpoint = m_refer_path[m_refer_path.size()-1];



	// for(int j=0;j<m_refer_path.size();j++)
	// {
	// 	std::cout<<"m_refer_path "<<m_refer_path[j].x<<" "<<m_refer_path[j].y<<" "<<m_refer_path[j].pointangle<<" ";
	// }

	// std::cout<<std::endl;
}

// static bool isfirst = true;
//判断机器人是否到达最终点，通过投影到先验路径方向看是否到达一定阈值之下
//并非单纯的计算集合距离
bool Bspline_Mtrajfinder::Rechgoal()
{
	std::cout<<"Goalpoint "<<Goalpoint.x<<" "<<Goalpoint.y<<std::endl;

	float GoalvectX = cos(Goalpoint.pointangle);
	float GoalvectY = sin(Goalpoint.pointangle);
	
	float VerticalGX;
	float VerticalGY;
	
	VerticalGX = -(GoalvectY/sqrt(GoalvectX*GoalvectX+GoalvectY*GoalvectY));
	VerticalGY = (GoalvectX/sqrt(GoalvectX*GoalvectX+GoalvectY*GoalvectY));
	
	
	float toGdisvectX;
	float toGdisvectY;
	
	toGdisvectX = (Rob.x - Goalpoint.x);
	toGdisvectY = (Rob.y - Goalpoint.y);
	
	float CostoGth;
	float toGth;
	
	float toGdisvectlen = sqrt(toGdisvectX*toGdisvectX+toGdisvectY*toGdisvectY);

	float VerticalGlen = sqrt(VerticalGX*VerticalGX+VerticalGY*VerticalGY);
	
	CostoGth = (toGdisvectX*VerticalGX + VerticalGY*toGdisvectY)/(toGdisvectlen*VerticalGlen);
	
	
	toGth = acos(CostoGth);
	
	
	float DistoGoal;
	
	DistoGoal = sqrt((Rob.x - Goalpoint.x)*(Rob.x - Goalpoint.x)+(Rob.y - Goalpoint.y)*(Rob.y - Goalpoint.y));
	

	if((DistoGoal*sin(toGth))<=2.0 && (refer_pathindex<m_refer_path.size() && refer_pathindex>(m_refer_path.size()-10)))
	{
		std::cout<<"reach goal "<<std::endl;
		return true;
	}

	// if(isfirst)
	// {
	// 	isfirst = false;
	// }
	return false;
}


//根据机器人全局位置和先验地图实时得到子目标点，再把子目标点转化的地图的坐标系中去
void Bspline_Mtrajfinder::Getsubgoal(std::string &frame_id,CPoint2d &subgoal)
{

	bool updateflag = false;

	std::cout<<"Getsubgoal now rob is "<<Rob.x
									<<" "<<Rob.y
									<<" "<<Rob.pointangle
									<<std::endl;
	std::cout<<"Getsubgoal last Rob is "<<lastRob.x
									<<" "<<lastRob.y
									<<" "<<lastRob.pointangle
									<<std::endl;
												
	CPoint2d ORsubgoal;
	
	if(codingstart)
	{
		lastRob = Rob;
		ORsubgoal = m_refer_path[refer_pathindex];
		
		
		if(frame_id=="/camera_init")
		{
			subgoal = ORsubgoal;
		}
		
		if(frame_id=="/rslidar")
		{
			//二维坐标系旋转平移
		
			subgoal = tansformWorldtoRob(ORsubgoal,Rob);
			
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
		
		codingstart = false;

		updateflag = true;
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
		
			subgoal = tansformWorldtoRob(ORsubgoal,Rob);
		}
		updateflag = true;
	}
	

	if(((Rob.x - lastRob.x) * (Rob.x - lastRob.x) + (Rob.y - lastRob.y) * (Rob.y - lastRob.y))>1.0)
	{
		std::cout<<"get in ^^^^^^^ "<<std::endl;
		lastRob = Rob;
		refer_pathindex = refer_pathindex + 1;
		if(refer_pathindex>=(m_refer_path.size()-1))
		{
			refer_pathindex = m_refer_path.size() - 1;
		}

		
		ORsubgoal = m_refer_path[refer_pathindex];
		
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
			
			subgoal = tansformWorldtoRob(ORsubgoal,Rob);
		}
	
		updateflag = true;
	}


	if(subgoal.Dist(Rob)<=4.0)
	{
		std::cout<<"xiaoyu subgoal.Dist(Rob)"<<std::endl;

		int disnum = subgoal.Dist(Rob);

		lastRob = Rob;
		refer_pathindex = refer_pathindex + 4 - disnum;

		if(refer_pathindex>=(m_refer_path.size()-1))
		{
			refer_pathindex = m_refer_path.size() - 1;
		}

		
		ORsubgoal = m_refer_path[refer_pathindex];
		
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
			
			subgoal = tansformWorldtoRob(ORsubgoal,Rob);
		}
	
		updateflag = true;
	}


	if(subgoal.Dist(Rob)>=6.0)
	{
		std::cout<<"dayu subgoal.Dist(Rob)"<<std::endl;

		int disnum = subgoal.Dist(Rob);

		for(int i=refer_pathindex;i>refer_pathindex-4;i--)
		{
			if(m_refer_path[i].Dist(Rob)<=5.5)
			{
				refer_pathindex = i;
				break;
			}
		}

		lastRob = Rob;

		if(refer_pathindex>=(m_refer_path.size()-1))
		{
			refer_pathindex = m_refer_path.size() - 1;
		}

		
		ORsubgoal = m_refer_path[refer_pathindex];
		
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
			
			subgoal = tansformWorldtoRob(ORsubgoal,Rob);
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
			subgoal = tansformWorldtoRob(ORsubgoal,Rob);
		}

	}

	std::cout<<"refer_pathindex "<<refer_pathindex<<std::endl;

	std::cout<<"ORsubgoal "<<ORsubgoal.x
									<<" "<<ORsubgoal.y
									<<" "<<ORsubgoal.pointangle
									<<std::endl;
	

	std::cout<<"subgoal.x"<<subgoal.x<<" "<<subgoal.y<<" "<<subgoal.pointangle<<std::endl;
	
	subGoal = subgoal;
	
}

//写下备用，把最后一个路径点再按最后它的方向扩展6m
//为了防止机器人快到最后的时候无法从先验路径上截到路径点
void Bspline_Mtrajfinder::ResizeGlobalpath()
{
	int globalpathsize = m_refer_path.size() - 1;
	
	CPoint2d refer_point = m_refer_path[globalpathsize];
	
	for(int i=1;i<7;i++)
	{
		CPoint2d tempoint;
		
		tempoint.x = refer_point.x + 
					 i * cos(refer_point.pointangle);
		
		tempoint.y = refer_point.y + 
					 i * sin(refer_point.pointangle);
		
		tempoint.pointangle = refer_point.pointangle;
		
		m_refer_path.push_back(tempoint);
	}
}


//将标准轨迹按照rightShift距离左右平移
void Bspline_Mtrajfinder::ChangePath(std::vector<CPoint2d> &oringinPath,std::vector<CPoint2d> &nowPath, float rightShift)
{
		std::vector<CPoint2d>().swap(nowPath);
		int size_originP = oringinPath.size();
		nowPath.reserve(size_originP);

		float anglePoint=0;
		anglePoint = atan2((oringinPath[2].x-oringinPath[1].x), (oringinPath[2].y-oringinPath[1].y));
		//anglePoint = atan2((oringinPath[size_originP-1].x-oringinPath[0].x), (oringinPath[size_originP-1].y-oringinPath[0].y));
		angletoEnd = anglePoint;
		anglePoint += 3.14159/2.0;
		anglePoint = anglePoint>(2*3.141159)?(anglePoint-2*3.142):anglePoint;
		/*std::cout<<"anglePoint anglePoint"<<anglePoint<<std::endl;*/
		for (int i=0;i<size_originP;i++)
		{
			//anglePoint = atan2((oringinPath[i+1].x-oringinPath[i].x), (oringinPath[i+1].y-oringinPath[i].y));
			////anglePoint = anglePoint<0.0?(anglePoint+2*3.14159):anglePoint;
			//anglePoint += 3.14159/2.0;
			//anglePoint = anglePoint>(2*3.141159)?(anglePoint-2*3.142):anglePoint;
			float nowPX = oringinPath[i].x+rightShift*sin(anglePoint);
			float nowPY = oringinPath[i].y+rightShift*cos(anglePoint);
			CPoint2d tempNowPoint(nowPX, nowPY);
			nowPath.push_back(tempNowPoint);
		}
}

//生成轨迹之前先把地图的数据保存成算法中使用的数据格式
//一个一维数组表示，1表示是障碍，-1表示可以，索引和地图索引是对应的
void Bspline_Mtrajfinder::Setmapindex(const nav_msgs::OccupancyGrid &msg)
{

	Mapreluton = msg.info.resolution;				//µØÍŒ·Ö±æÂÊ
	Map_originPx = msg.info.origin.position.x;             //µØÍŒÆðÊŒÈ«ŸÖ×ø±ê
	Map_originPy = msg.info.origin.position.y;				//µØÍŒÆðÊŒÈ«ŸÖ×ø±ê
	Mapheight = msg.info.height;					//µØÍŒžß¶ÈºÍ¿í¶È
	Mapwidth = msg.info.width;


	MapObindex.assign(msg.info.width*msg.info.height,-1);
	
	int mapsize = sqrt(Ob_thread)/msg.info.resolution+1;
	
	for(int ny=0; ny<msg.info.height; ny++)
	{
		for(int nx=0; nx<msg.info.width; nx++)
		{
			int mapindex = ny * msg.info.width + nx;
			
			if((unsigned char)msg.data[mapindex]>=120)
			{
				//确定该障碍栅格之后再确定周围多少临域内的栅格是障碍的
				for(int i=(-mapsize);i<=mapsize;i++)
				{
					for(int j=(-mapsize);j<=mapsize;j++)
					{
						// std::cout<<" i "<<i<<" j "<<j<<std::endl;
						float judegeObdis = (i*i+j*j)*(msg.info.resolution*msg.info.resolution);
						if(i==0 && j==0)
						{
							int Obindex = msg.info.width*ny+nx;
							MapObindex[Obindex] = 1;
							// std::cout<<"--- obcenter --- "<<Obindex;
							continue;
						}
						if(judegeObdis<=Ob_thread)
						{
							int Obindex = msg.info.width*(ny+i)+(nx+j);
							if(Obindex>=0 && Obindex<=(msg.info.width*msg.info.height-1))
							{
								// std::cout<<"--- obcenter zhouwei ---- "<<Obindex;
								MapObindex[Obindex] = 1;
							}
						}
					}
				}			
			}
		}
	}
}

//检查一条轨迹上的点是否存在障碍函数
//----该函数是利用将轨迹点投入局部地图栅格里判断栅格属性的方法
bool Bspline_Mtrajfinder::Checkpath(std::vector<CPoint2d> &potential_paths,std::vector<int> &temMapObindex)
{
	for(int i=0;i<potential_paths.size();i++)
	{
		float dx_f,dy_f;
		int colx,rowy;
		dx_f = potential_paths[i].x - Map_originPx;    
        dy_f = potential_paths[i].y - Map_originPy;
        colx = lround(dx_f / Mapreluton);
        rowy = lround(dy_f / Mapreluton);
        int Checkindex = Mapwidth*rowy+colx;
        if(MapObindex[Checkindex] == 1)
        {
        	return false;    	
        }
	}
	return true;

}

//在所有可行路径中根据优先级规定的距离去判断可行路径里边有没有位于这个区域的轨迹
bool Bspline_Mtrajfinder::chosepath(std::vector<pro_path> &BeCheckallpath,const float &dis,int &num)
{
	// std::cout<<"dis dis "<<dis<<std::endl;
    for(int i=0;i<BeCheckallpath.size();i++)
    {
    	if(((BeCheckallpath[i].distorightline-dis)*(BeCheckallpath[i].distorightline-dis))<=0.06)
    	{
			
			if(show_log)
			{
				std::cout<<"(BeCheckallpath[i].distorightline "<<BeCheckallpath[i].distorightline<<std::endl;
    		}
			
			num = i;
    		return true;
    	}
    }
    return false;
}

//根据之前确定的验先验路径还是靠右方式的距离向两边按照优先顺序
//遍历所有可行路径，找到最优的
void Bspline_Mtrajfinder::chosetraj(std::vector<pro_path> &BeCheckallpath,
					const float &m_shiftpath,int &BeCheckallpathindex,const float &unitShift)
{

    bool checksuss = false;

	if(Strategy_level == 2)		//靠中线方式轨迹最右轨迹选择方式，利用基准距离开始选择轨迹
	{
		for(float j=0;j<=m_roadWidth*2;)
	    {
	    	// std::cout<<"----- m_shiftpath----- "<<m_shiftpath<<std::endl;
	    	// std::cout<<"---j--- "<<j<<std::endl;
	    	if(j!=0)
	    	{
	    		if((m_shiftpath-j)>=-(m_roadWidth*2))
	    		{
	    			// std::cout<<"enter m_shiftpath-j_________ "<<m_shiftpath-j<<std::endl;
	    			for(float i=0;i<(unitShift*intervaltrajNum);)
	    			{
	    				// std::cout<<"m_shiftpath-j-i "<<m_shiftpath-j-i<<std::endl;

	    				if(chosepath(BeCheckallpath,m_shiftpath-j-i,BeCheckallpathindex))
	    				{
							
							if(show_log)
							{
								std::cout<<"m_shiftpath-j-i SUCCESSS "<<m_shiftpath-j-i<<std::endl;
							}
							
	    					checksuss = true;
	    					break;
	    				}
	    				i = i + unitShift;
	    			}

	    			if(checksuss)
	    			{
						if(show_log)
						{
							std::cout<<"m_shiftpath-j-i SUCCESSS "<<std::endl;
						}
	    				break;
	    			}
					
					if(show_log)
					{
						std::cout<<"m_shiftpath-j-i NO SUCCESSS"<<std::endl;
					}
					
	    		}

	    		if((m_shiftpath+j)<=(m_roadWidth*2))
	    		{
	    			// std::cout<<"enter m_shiftpath+j_________ "<<m_shiftpath+j<<std::endl;

	    			for(float i=0;i<(unitShift*intervaltrajNum);)
	    			{	
	    				// std::cout<<"m_shiftpath+j+i "<<m_shiftpath+j+i<<std::endl;
	    				if(chosepath(BeCheckallpath,m_shiftpath+j+i,BeCheckallpathindex))
	    				{
							if(show_log)
							{
								std::cout<<"m_shiftpath+j+i SUCCESSS "<<m_shiftpath+j+i<<std::endl;
							}
							
	    					checksuss = true;
	    					break;
	    				}
	    				i = i + unitShift;
	    			}
	    			if(checksuss)
	    			{
						if(show_log)
						{
							std::cout<<"m_shiftpath+j SUCCESSS "<<std::endl;
						}
						
	    				break;
	    			}
					if(show_log)
					{
						std::cout<<"m_shiftpath+j NO SUCCESSS"<<std::endl;
					}						
	    		}
	    		
	    	}
	    	else
	    	{
	    		// std::cout<<"enter m_shiftpath_____________ "<<m_shiftpath<<std::endl;

	    		if(chosepath(BeCheckallpath,m_shiftpath,BeCheckallpathindex))
	    		{
					if(show_log)
					{
						std::cout<<"m_shiftpath SUCCESSS "<<m_shiftpath<<std::endl;
					}
	    			break;
	    		}
				
				if(show_log)
				{
					std::cout<<"m_shiftpath NO SUCCESSS"<<std::endl;
				}

	    		for(float i=unitShift;i<(unitShift*intervaltrajNum);)
	    		{
	    			// std::cout<<"m_shiftpath-j-i "<<m_shiftpath-j-i<<std::endl;

	    			if(chosepath(BeCheckallpath,m_shiftpath-j-i,BeCheckallpathindex))
	    			{
						
						if(show_log)
						{
							std::cout<<"m_shiftpath-j-i SUCCESSS "<<m_shiftpath-j-i<<std::endl;
						}
						
	    				checksuss = true;
	    				break;
	    				
	    			}

	    			i = i + unitShift;
	    		}

	    		if(checksuss)
	    		{
					if(show_log)
					{
						std::cout<<"m_shiftpath-j-i SUCCESSS "<<std::endl;
					}
					
	    			break;
	    		}

	    		for(float i=unitShift;i<(unitShift*intervaltrajNum);)
	    		{	
	    			// std::cout<<"m_shiftpath+j+i "<<m_shiftpath+j+i<<std::endl;
	    			if(chosepath(BeCheckallpath,m_shiftpath+j+i,BeCheckallpathindex))
	    			{
						if(show_log)
						{
							std::cout<<"m_shiftpath+j+i SUCCESSS "<<m_shiftpath+j+i<<std::endl;
						}
						
	    				checksuss = true;
	    				break;
	    				
	    			}
	    			i = i + unitShift;
	    		}

	    		if(checksuss)
	    		{
					if(show_log)
					{
						std::cout<<"m_shiftpath+j+i SUCCESSS "<<std::endl;
					}
					
	    			break;
	    		}
	    	}	

	    	j=j+unitShift*intervaltrajNum;
	    }

	}
	else	//不是中线方式最优轨迹选择方式，利用基准距离开始选择轨迹
	{
	
		for(float j=0;j<=m_roadWidth*2;)
	    {
	    	// std::cout<<"----- m_shiftpath----- "<<m_shiftpath<<std::endl;
	    	// std::cout<<"---j--- "<<j<<std::endl;
	    	if(j!=0)
	    	{
	    		if((m_shiftpath+j)<=(m_roadWidth*2))
	    		{
	    			// std::cout<<"enter m_shiftpath+j_________ "<<m_shiftpath+j<<std::endl;

	    			for(float i=0;i<(unitShift*intervaltrajNum);)
	    			{	
	    				// std::cout<<"m_shiftpath+j+i "<<m_shiftpath+j+i<<std::endl;
	    				if(chosepath(BeCheckallpath,m_shiftpath+j+i,BeCheckallpathindex))
	    				{
							if(show_log)
							{
								std::cout<<"m_shiftpath+j+i SUCCESSS "<<m_shiftpath+j+i<<std::endl;
							}

							
	    					checksuss = true;
	    					break;
	    				}
	    				i = i + unitShift;
	    			}
	    			if(checksuss)
	    			{
						if(show_log)
						{
							std::cout<<"m_shiftpath+j SUCCESSS "<<std::endl;
						}
						
	    				break;
	    			}
					if(show_log)
					{
						std::cout<<"m_shiftpath+j NO SUCCESSS"<<std::endl;
					}						
	    		}

	    		if((m_shiftpath-j)>=-(m_roadWidth*2))
	    		{


	    			// std::cout<<"m_shiftpath-j+unitShift*intervaltrajNum "<<m_shiftpath-j+unitShift*intervaltrajNum<<std::endl;

	    			// for(float i=0;i<(unitShift*intervaltrajNum);)
	    			// {
	    			// 	std::cout<<"m_shiftpath-j-i "<<m_shiftpath-j-i+unitShift*intervaltrajNum<<std::endl;

	    			// 	if(chosepath(BeCheckallpath,m_shiftpath-j-i+unitShift*intervaltrajNum,BeCheckallpathindex))
	    			// 	{
	    			// 		std::cout<<"m_shiftpath-j-i SUCCESSS "<<m_shiftpath-j-i+unitShift*intervaltrajNum<<std::endl;
	    			// 		checksuss = true;
	    			// 		break;
	    			// 	}
	    			// 	i = i + unitShift;
	    			// }


	    			// std::cout<<"enter m_shiftpath-j_________ "<<m_shiftpath-j<<std::endl;
	    			for(float i=0;i<(unitShift*intervaltrajNum);)
	    			{
	    				// std::cout<<"m_shiftpath-j-i "<<m_shiftpath-j-i<<std::endl;

	    				if(chosepath(BeCheckallpath,m_shiftpath-j-i,BeCheckallpathindex))
	    				{
							if(show_log)
							{
								std::cout<<"m_shiftpath-j-i SUCCESSS "<<m_shiftpath-j-i<<std::endl;
							}
							
	    					checksuss = true;
	    					break;
	    				}
	    				i = i + unitShift;
	    			}

	    			if(checksuss)
	    			{
						if(show_log)
						{
							std::cout<<"m_shiftpath-j-i SUCCESSS "<<std::endl;
						}
						
	    				break;
	    			}
					
					if(show_log)
					{
						std::cout<<"m_shiftpath-j-i NO SUCCESSS"<<std::endl;
					}
	    		}

	    	}

	    	else
	    	{
	    		// std::cout<<"enter m_shiftpath_____________ "<<m_shiftpath<<std::endl;

	    		if(chosepath(BeCheckallpath,m_shiftpath,BeCheckallpathindex))
	    		{
					if(show_log)
					{
						std::cout<<"m_shiftpath SUCCESSS "<<m_shiftpath<<std::endl;
					}
	    			break;
	    		}
				
				if(show_log)
				{
					std::cout<<"m_shiftpath NO SUCCESSS"<<std::endl;
				}

	    		for(float i=unitShift;i<(unitShift*intervaltrajNum);)
	    		{	
	    			// std::cout<<"m_shiftpath+j+i "<<m_shiftpath+j+i<<std::endl;
	    			if(chosepath(BeCheckallpath,m_shiftpath+j+i,BeCheckallpathindex))
	    			{
						
						if(show_log)
						{
							std::cout<<"m_shiftpath+j+i SUCCESSS "<<m_shiftpath+j+i<<std::endl;
	    				}
						
						checksuss = true;
	    				break;
	    				
	    			}
	    			i = i + unitShift;
	    		}

	    		if(checksuss)
	    		{
					if(show_log)
					{
						std::cout<<"m_shiftpath+j+i SUCCESSS "<<std::endl;
	    			}
					break;
	    		}

	    		for(float i=unitShift;i<(unitShift*intervaltrajNum);)
	    		{
	    			// std::cout<<"m_shiftpath-j-i "<<m_shiftpath-j-i<<std::endl;

	    			if(chosepath(BeCheckallpath,m_shiftpath-j-i,BeCheckallpathindex))
	    			{
						if(show_log)
						{
							std::cout<<"m_shiftpath-j-i SUCCESSS "<<m_shiftpath-j-i<<std::endl;
	    				}
						checksuss = true;
	    				break;
	    				
	    			}

	    			i = i + unitShift;
	    		}

	    		if(checksuss)
	    		{
					if(show_log)
					{
						std::cout<<"m_shiftpath-j-i SUCCESSS "<<std::endl;
	    			}
					break;
	    		}

	    	}	

	    	j=j+unitShift*intervaltrajNum;
	    }
	}
	
	if(show_log)
	{
		std::cout<<"BeCheckallpathindex "<<BeCheckallpathindex<<std::endl;
	}	
}

//生成轨迹函数
void Bspline_Mtrajfinder::Generatraj(std::string &frame_id)
{
	
	LG_Paths.clear();
	LG_PathsCopy.clear();
	RG_Paths.clear();
	RG_PathsCopy.clear();
	LGJudgeOb_paths.clear();
    RGJudgeOb_paths.clear();
	CLbecheck_paths.clear();
	Rbecheck_paths.clear();
	partreferpath.clear();
	
	//得到子目标点，然后去先验路径上截取相应的路径作为基准
	std::cout<<"subgoal is "<<subGoal.x<<" "<<subGoal.y<<" "<<subGoal.pointangle<<std::endl;
	std::vector<CPoint2d> m_pathNeedCheck;
	float finalTh = subGoal.pointangle;
	
	//截取目前子目标点所以向后的5个点作为基准路径
	// refer_pathindex初值为6
	int istart = refer_pathindex - 5;
	
	for(int i = istart;i<=refer_pathindex;i++)
	{
		m_pathNeedCheck.push_back(m_refer_path[i]);
	} 
	
	
	std::vector<CPoint2d> local_m_pathNeedCheck;
	
	if(frame_id == "/rslidar")
	{
		//将全局路径点转化至机器人的局部坐标系中，在局部坐标系进行
		//轨迹生成和规划
		
		for(int j=0;j<m_pathNeedCheck.size();j++)
		{
			CPoint2d temp;
			temp = tansformWorldtoRob(m_pathNeedCheck[j],Rob);
			local_m_pathNeedCheck.push_back(temp);
		}

		//用于存储当前截取到的先验路径上的点，用于之后提供给中线策略方法使用
		for(int i=0;i<local_m_pathNeedCheck.size();i++)
		{
			partreferpath.push_back(local_m_pathNeedCheck[i]);
		}

		CPoint2d firstPoint;
		firstPoint.x = 0.0;
		firstPoint.y = 0.0;
		
		std::vector<CPoint2d> newrefer_path;
		for(int i=0;i<local_m_pathNeedCheck.size();i++)
		{
			CPoint2d addExPoint;
			addExPoint.x = firstPoint.x + 
						   1*cos(local_m_pathNeedCheck[i].pointangle);
			
			addExPoint.y = firstPoint.y + 
						   1*sin(local_m_pathNeedCheck[i].pointangle);
			
			firstPoint = addExPoint;
			
			newrefer_path.push_back(addExPoint);
		}
		
		//如果截到的路径不够6个，通过最后一个点的角度自动补全6个
		if(newrefer_path.size()<4)
		{
			int neednum = 6- newrefer_path.size();
			for(int j=0;j< neednum;j++)
			{
				CPoint2d addPoint;
				addPoint.x=newrefer_path[newrefer_path.size()-1].x+(j+1)*cos(finalTh);
				addPoint.y=newrefer_path[newrefer_path.size()-1].y+(j+1)*sin(finalTh);
				newrefer_path.push_back(addPoint);
			}

		}

		local_m_pathNeedCheck = newrefer_path;
		
		int Checkpathnum = local_m_pathNeedCheck.size();
		
		std::vector<CPoint2d> tempm_path;
		CPoint2d JubuRob;
		JubuRob.x = 0.0;
		JubuRob.y = 0.0;
		JubuRob.pointangle = 0.0;
		
		for(int j=0;j<=Checkpathnum;j++)
		{
			if(j==0)
			{
				tempm_path.push_back(JubuRob);
			}
			else
			{

				tempm_path.push_back(local_m_pathNeedCheck[j-1]);
			}
		}
		
		std::vector<CPoint2d> ().swap(local_m_pathNeedCheck);
		local_m_pathNeedCheck = tempm_path;
	}
	
	
	if(frame_id == "/camera_init")
	{

		//用于存储当前截取到的先验路径上的点，用于之后提供给中线策略方法使用
		for(int i=0;i<m_pathNeedCheck.size();i++)
		{
			partreferpath.push_back(m_pathNeedCheck[i]);
		}


		//将全局坐标系下的标准路径
		//轨迹生成和规划
		
		CPoint2d firstPoint;
		firstPoint.x = Rob.x;
		firstPoint.y = Rob.y;
		
		std::vector<CPoint2d> newrefer_path;
		for(int i=0;i<m_pathNeedCheck.size();i++)
		{
			CPoint2d addExPoint;
			addExPoint.x = firstPoint.x + 
						   1*cos(m_pathNeedCheck[i].pointangle);
			
			addExPoint.y = firstPoint.y + 
						   1*sin(m_pathNeedCheck[i].pointangle);
			
			firstPoint = addExPoint;
			
			newrefer_path.push_back(addExPoint);
		}
		
		//如果截到的路径不够6个，通过最后一个点的角度自动补全6个
		if(newrefer_path.size()<4)
		{
			int neednum = 6- newrefer_path.size();
			for(int j=0;j< neednum;j++)
			{
				CPoint2d addPoint;
				addPoint.x=newrefer_path[newrefer_path.size()-1].x+(j+1)*cos(finalTh);
				addPoint.y=newrefer_path[newrefer_path.size()-1].y+(j+1)*sin(finalTh);
				newrefer_path.push_back(addPoint);
			}

		}

		m_pathNeedCheck = newrefer_path;
		
		int Checkpathnum = m_pathNeedCheck.size();
		
		std::vector<CPoint2d> tempm_path;
		CPoint2d JubuRob;
		JubuRob.x = Rob.x;
		JubuRob.y = Rob.y;
		JubuRob.pointangle = Rob.pointangle;
		
		for(int j=0;j<=Checkpathnum;j++)
		{
			if(j==0)
			{
				tempm_path.push_back(JubuRob);
			}
			else
			{

				tempm_path.push_back(m_pathNeedCheck[j-1]);
			}
		}
		
		std::vector<CPoint2d> ().swap(m_pathNeedCheck);
		m_pathNeedCheck = tempm_path;
		
	}
	

	std::cout<<"m_pathNeedCheck "<<m_pathNeedCheck.size()<<std::endl;
	
	std::cout<<"local_m_pathNeedCheck "<<local_m_pathNeedCheck.size()<<std::endl;
	
	
	
	float m_standardShift = 0;
	
	CPoint2d start_Fist;
	
	if(frame_id == "/rslidar")
	{
		start_Fist.x = 0.0;
		start_Fist.y = 0.0;
		start_Fist.pointangle = 0.0; 
	}
	
	if(frame_id == "/camera_init")
	{		
		start_Fist.x = Rob.x;
		start_Fist.y = Rob.y;
		start_Fist.pointangle = Rob.pointangle; 
		
	}
	
	

	
	float unitShift = m_roadWidth/roadways; 

	std::cout<<" unitShift "<<unitShift<<std::endl;
	
	//通过截取得到的路径为基准以unitShift的间隔向两边进行平移
	
	//left first then right
	for (float i=0.0;i<(m_roadWidth/2+m_roadWidth);)
	{
		// std::cout<<"i "<<i<<std::endl;
		std::vector<CPoint2d> m_pathRShift;
		
		if(i<0.001)
		{	
			// std::cout<<"iiiiiiii "<<i<<std::endl;

			if(flagbsplinesmooth)
			{
				// std::cout<<"shift shift shift  "<<m_standardShift<<std::endl;

				
				if(frame_id == "/rslidar")
				{
					ChangePath(local_m_pathNeedCheck, m_pathRShift, m_standardShift);
				}
				
				if(frame_id == "/camera_init")
				{
					ChangePath(m_pathNeedCheck, m_pathRShift, m_standardShift);
				}
					
				
				m_pathRShift[0]=start_Fist;
					
				LG_PathsCopy.push_back(m_pathRShift);

				CBSpline m_pathGener(3, m_pathRShift);
				std::vector<CPoint2d> ().swap(m_pathRShift);
				m_pathRShift = m_pathGener.GetBSplinePoint();

				std::vector<CPoint2d> m_pathRShift_Copy;
				for (int i=0;i<m_pathRShift.size();)
				{
					m_pathRShift_Copy.push_back(m_pathRShift[i]);
					i += (m_pathRShift.size()/sampnum);
				}//get 20 traj points after smooth

				std::vector<CPoint2d> ().swap(m_pathRShift);
				m_pathRShift = m_pathRShift_Copy;

				LG_Paths.push_back(m_pathRShift);
				
			}		
	
			if(flagbazersmooth)	
			{

				// std::cout<<"shift shift shift  "<<m_standardShift-i<<std::endl;

				std::vector<CPoint2d> Bespath;
					
				Bespath.push_back(start_Fist);
				
				if(frame_id == "/rslidar")
				{
					Bespath.push_back(local_m_pathNeedCheck[1]);
					
					ChangePath(local_m_pathNeedCheck, m_pathRShift, m_standardShift-i);
				}
				
				if(frame_id == "/camera_init")
				{
					Bespath.push_back(m_pathNeedCheck[1]);
					
					ChangePath(m_pathNeedCheck, m_pathRShift, m_standardShift-i);
					
				}
				
				
				m_pathRShift[0]=start_Fist;

				LG_PathsCopy.push_back(m_pathRShift);
					
				for(int i=2;i<m_pathRShift.size();i++)
				{
					Bespath.push_back(m_pathRShift[i]);
				}
					
				std::vector<CPoint2d>().swap(m_pathRShift);

				// std::cout<<"Bespath "<<Bespath.size()<<std::endl;
					
				m_pathRShift = Bezier(Bespath,10);

				LG_Paths.push_back(m_pathRShift);
				
			}
		}

		if (i!=0.0)
		{
			// std::cout<<"m_standardShift-i "<<m_standardShift-i<<std::endl;
			// std::cout<<"m_standardShift+i "<<m_standardShift+i<<std::endl;
			//left shift
			if ((m_standardShift-i)>=-(m_roadWidth/2+extra_roadWidth))
			{
				
				if(flagbsplinesmooth)
				{
					// std::cout<<"shift shift shift  "<<m_standardShift-i<<std::endl;
					
					if(frame_id == "/rslidar")
					{
						ChangePath(local_m_pathNeedCheck, m_pathRShift, m_standardShift-i);
					}
					
					if(frame_id == "/camera_init")
					{
						ChangePath(m_pathNeedCheck, m_pathRShift, m_standardShift-i);
					}
					

					m_pathRShift[0]=start_Fist;

					LG_PathsCopy.push_back(m_pathRShift);

					CBSpline m_pathGenerL(3, m_pathRShift);
					std::vector<CPoint2d> ().swap(m_pathRShift);
					m_pathRShift = m_pathGenerL.GetBSplinePoint();
					std::vector<CPoint2d> m_pathRShift_Copy;

					for (int i=0;i<m_pathRShift.size();)
					{
						m_pathRShift_Copy.push_back(m_pathRShift[i]);
						i += (m_pathRShift.size()/sampnum);
					}//get 20 traj points after smooth

					std::vector<CPoint2d>().swap(m_pathRShift);
					m_pathRShift = m_pathRShift_Copy;

					LG_Paths.push_back(m_pathRShift);
				}
				
				if(flagbazersmooth)
				{
				// 	std::cout<<"shift shift shift  "<<m_standardShift-i<<std::endl;

					std::vector<CPoint2d> Bespath;
					
					
					Bespath.push_back(start_Fist);
					
					if(frame_id == "/rslidar")
					{
						Bespath.push_back(local_m_pathNeedCheck[1]);
					
						ChangePath(local_m_pathNeedCheck, m_pathRShift, m_standardShift-i);
						
					}
					
					if(frame_id == "/camera_init")
					{
						Bespath.push_back(m_pathNeedCheck[1]);
					
						ChangePath(m_pathNeedCheck, m_pathRShift, m_standardShift-i);
					
					}
					

					m_pathRShift[0]=start_Fist;

					LG_PathsCopy.push_back(m_pathRShift);
					
					for(int i=2;i<m_pathRShift.size();i++)
					{
						Bespath.push_back(m_pathRShift[i]);
					}

					// std::cout<<"Bespath "<<Bespath.size()<<std::endl;
					
					std::vector<CPoint2d>().swap(m_pathRShift);
					
					m_pathRShift = Bezier(Bespath,10);

					LG_Paths.push_back(m_pathRShift);
					
				}
			}

			//right shif
			if ((m_standardShift+i)<=(m_roadWidth/2+extra_roadWidth))
			{

				if(flagbsplinesmooth)
				{
					// std::cout<<"shift shift shift  "<<m_standardShift+i<<std::endl;

					std::vector<CPoint2d>().swap(m_pathRShift);
					
					if(frame_id == "/rslidar")
					{
						ChangePath(local_m_pathNeedCheck, m_pathRShift, m_standardShift+i);
					}
					
					if(frame_id == "/camera_init")
					{
						ChangePath(m_pathNeedCheck, m_pathRShift, m_standardShift+i);
					}

					m_pathRShift[0]=start_Fist;

					RG_PathsCopy.push_back(m_pathRShift);

					CBSpline m_pathGenerR(3, m_pathRShift);
					std::vector<CPoint2d> ().swap(m_pathRShift);
					m_pathRShift = m_pathGenerR.GetBSplinePoint();

					std::vector<CPoint2d> m_pathRShift_Copy;
					for (int i=0;i<m_pathRShift.size();)
					{
						m_pathRShift_Copy.push_back(m_pathRShift[i]);
						i += (m_pathRShift.size()/sampnum);
					}//get 20 traj points after smooth

					std::vector<CPoint2d> ().swap(m_pathRShift);
					m_pathRShift = m_pathRShift_Copy;

					RG_Paths.push_back(m_pathRShift);
					
				}
				
				
				if(flagbazersmooth)
				{
					// std::cout<<"shift shift shift  "<<m_standardShift+i<<std::endl;
					
					std::vector<CPoint2d> Bespath;
					Bespath.push_back(start_Fist);
					
					if(frame_id == "/rslidar")
					{
						Bespath.push_back(local_m_pathNeedCheck[1]);
						std::vector<CPoint2d>().swap(m_pathRShift);
						ChangePath(local_m_pathNeedCheck, m_pathRShift, m_standardShift+i);	
					}
					
					if(frame_id == "/camera_init")
					{
						Bespath.push_back(m_pathNeedCheck[1]);
						std::vector<CPoint2d>().swap(m_pathRShift);
						ChangePath(m_pathNeedCheck, m_pathRShift, m_standardShift+i);					
					}
					

					m_pathRShift[0]=start_Fist;

					RG_PathsCopy.push_back(m_pathRShift);
					
					for(int i=2;i<m_pathRShift.size();i++)
					{
						Bespath.push_back(m_pathRShift[i]);
					}
					
					// std::cout<<"Bespath "<<Bespath.size()<<std::endl;

					std::vector<CPoint2d>().swap(m_pathRShift);
					
					m_pathRShift = Bezier(Bespath,10);
					
					RG_Paths.push_back(m_pathRShift);
					
				}

			}
		}

		i = i + unitShift;
	}
	
	std::cout<<"left shift traj num is  "<<LG_Paths.size()<<std::endl;

	std::cout<<"right shift traj num is "<<RG_Paths.size()<<std::endl;
	
	//从平滑之后的路径中截取在之前截取的先验路径上一定距离投影的路径
	//作为判断是否接触到障碍物使用
	for(int j=0;j<LG_Paths.size();j++)
	{
		CPoint2d robotPos;

		if(frame_id == "/rslidar")
		{
			robotPos.x = 0.0;
			robotPos.y = 0.0;
		}
		
		if(frame_id == "/camera_init")
		{
			robotPos.x = Rob.x;
			robotPos.y = Rob.y;
		}
		
		// std::cout<<"ob judge robpose "<<robotPos.x<<" "<<robotPos.y<<std::endl;
		// std::cout<<"ob judge angletoEnd "<<angletoEnd<<std::endl;
		std::vector<CPoint2d> Judge_paths;
		Judge_paths.push_back(robotPos);
		for(int i=1;i<LG_Paths[j].size();i++)
		{
			float dis=0;

			dis = robotPos.Dist(LG_Paths[j][i]);

			// std::cout<<"position "<<potential_paths[i].y<<" "<<potential_paths[i].x<<" "<<std::endl;
			// std::cout<<"robotPos "<<robotPos.y<<" "<<robotPos.x<<" "<<std::endl;

			float Y = LG_Paths[j][i].y - start_Fist.y;
			float X = LG_Paths[j][i].x - start_Fist.x;

			// std::cout<<"X Y "<<X<<" "<<Y<<" "<<std::endl;


			float toRobangle = atan2(X,	Y);


			// std::cout<<"toRobangle "<<toRobangle<<std::endl;

			float angleFrobtpa;
			angleFrobtpa = acos(cos(toRobangle)*cos(angletoEnd)+sin(toRobangle)*sin(angletoEnd));

			float tompathdis=dis*cos(angleFrobtpa);

			// std::cout<<"Dis "<<dis<<" "<<tompathdis<<" "<<angleFrobtpa<<std::endl;

			//两种平滑路径截取不同长度的轨迹作为有效轨迹
			if(flagbsplinesmooth)
			{
				if(tompathdis<=judgeOblen)
				{
					Judge_paths.push_back(LG_Paths[j][i]);
				}
				else
				{
					continue;
				}
			}
			//两种平滑路径截取不同长度的轨迹作为有效轨迹
			if(flagbazersmooth)
			{
				judgeOblen = 5.0;
				if(tompathdis<=judgeOblen)
				{
					Judge_paths.push_back(LG_Paths[j][i]);
				}
				else
				{
					continue;
				}
			}

		}

		LGJudgeOb_paths.push_back(Judge_paths);
	}

	for(int j=0;j<RG_Paths.size();j++)
	{
		CPoint2d robotPos;

		if(frame_id == "/rslidar")
		{
			robotPos.x = 0.0;
			robotPos.y = 0.0;
		}
		
		if(frame_id == "/camera_init")
		{
			robotPos.x = Rob.x;
			robotPos.y = Rob.y;
		}
		// std::cout<<"ob judge robpose "<<robotPos.x<<" "<<robotPos.y<<std::endl;
		// std::cout<<"ob judge angletoEnd "<<angletoEnd<<std::endl;
		std::vector<CPoint2d> Judge_paths;
		Judge_paths.push_back(robotPos);
		for(int i=1;i<RG_Paths[j].size();i++)
		{
			float dis=0;

			dis = robotPos.Dist(RG_Paths[j][i]);

			// std::cout<<"position "<<potential_paths[i].y<<" "<<potential_paths[i].x<<" "<<std::endl;
			// std::cout<<"robotPos "<<robotPos.y<<" "<<robotPos.x<<" "<<std::endl;

			float Y = RG_Paths[j][i].y - start_Fist.y;;
			float X = RG_Paths[j][i].x - start_Fist.x;

			// std::cout<<"X Y "<<X<<" "<<Y<<" "<<std::endl;


			float toRobangle = atan2(X,	Y);


			// std::cout<<"toRobangle "<<toRobangle<<std::endl;

			float angleFrobtpa;
			angleFrobtpa = acos(cos(toRobangle)*cos(angletoEnd)+sin(toRobangle)*sin(angletoEnd));

			float tompathdis=dis*cos(angleFrobtpa);

			// std::cout<<"Dis "<<dis<<" "<<tompathdis<<" "<<angleFrobtpa<<std::endl;

			//两种平滑路径截取不同长度的轨迹作为有效轨迹
			if(flagbsplinesmooth)
			{
				if(tompathdis<=judgeOblen)
				{
					Judge_paths.push_back(RG_Paths[j][i]);
				}
				else
				{
					continue;
				}
			}
			//两种平滑路径截取不同长度的轨迹作为有效轨迹
			if(flagbazersmooth)
			{
				judgeOblen = 5.0;
				if(tompathdis<=judgeOblen)
				{
					Judge_paths.push_back(RG_Paths[j][i]);
				}
				else
				{
					continue;
				}
			}
		}

		RGJudgeOb_paths.push_back(Judge_paths);
	}

	std::cout<<"LGJudgeOb_paths.size() "<<LGJudgeOb_paths.size()<<std::endl;
	std::cout<<"RGJudgeOb_paths.size() "<<RGJudgeOb_paths.size()<<std::endl;

	
	//获得要判断障碍的轨迹集合之后，开始进行障碍物检测，删除掉那些与障碍物有接触的轨迹
	//通过轨迹记录下距离标准轨迹的距离，然后是左右的第几条轨迹
	
	for(int i=0;i<LGJudgeOb_paths.size();i++)
	{
		if(Checkpath(LGJudgeOb_paths[i],MapObindex))//取出可行轨迹，并计算可行轨迹的属性--是机器人前段轨迹靠右靠左和距离
		{
			pro_path temppro_path;
			temppro_path.Ex_path = LGJudgeOb_paths[i];
			temppro_path.pathNum = i;
			temppro_path.LeoRit = 0;
			temppro_path.distorightline = -unitShift*i;
			CLbecheck_paths.push_back(temppro_path);
		}
	}

	for(int i=0;i<RGJudgeOb_paths.size();i++)
	{
		if(Checkpath(RGJudgeOb_paths[i],MapObindex))//取出可行轨迹，并计算可行轨迹的属性--是机器人前段轨迹靠右靠左和距离
		{
			pro_path temppro_path;
			temppro_path.Ex_path = RGJudgeOb_paths[i];
			temppro_path.pathNum = i+1;
			temppro_path.LeoRit = 1;
			temppro_path.distorightline = unitShift*(i+1);
			Rbecheck_paths.push_back(temppro_path);
		}
	}
	
	std::cout<<"CLbecheck_paths.size() "<<CLbecheck_paths.size()<<std::endl;
	std::cout<<"Rbecheck_paths.size() "<<Rbecheck_paths.size()<<std::endl;
	
	if(show_log)
	{
		for(int i=0;i<CLbecheck_paths.size();i++)
		{
			std::cout<<CLbecheck_paths[i].pathNum<<" "<<CLbecheck_paths[i].LeoRit<<" "<<CLbecheck_paths[i].distorightline<<"---- ";
		}  
		std::cout<<std::endl;  

		for(int i=0;i<Rbecheck_paths.size();i++)
		{
			std::cout<<Rbecheck_paths[i].pathNum<<" "<<Rbecheck_paths[i].LeoRit<<" "<<Rbecheck_paths[i].distorightline<<"---- ";
		}  
		std::cout<<std::endl;  
	}
	
	//将可行轨迹的距离转化为距离最右端轨迹的距离
	//对轨迹进行编码，从最右的轨迹为第一条，
	//记录下距离最右轨迹的距离、编号

	int endpathnum;
	float centerdis;
	float m_shiftpath;
	float endpathdis;

	std::vector<pro_path> BeCheckallpath;

	if(!Rbecheck_paths.empty())
	{
  		endpathnum = Rbecheck_paths.size()-1;
  		if(!CLbecheck_paths.empty())
  		{
  		    endpathdis = Rbecheck_paths[endpathnum].distorightline;
			
			if(show_log)
			{
				std::cout<<"left and right "<<endpathdis<<"  endpathnum "<<endpathnum<<std::endl;
			}
			
  			for(int i=(Rbecheck_paths.size()-1);i>=0;i--)
  			{
  				Rbecheck_paths[i].distorightline = endpathdis - Rbecheck_paths[i].distorightline;
  				BeCheckallpath.push_back(Rbecheck_paths[i]);
  			}
  			for(int i=0;i<CLbecheck_paths.size();i++)
  			{
  				CLbecheck_paths[i].distorightline =  endpathdis - CLbecheck_paths[i].distorightline;
  				BeCheckallpath.push_back(CLbecheck_paths[i]);
  			}
  			centerdis = CLbecheck_paths[CLbecheck_paths.size()-1].distorightline/2;
  		}
  		else
  		{
  			endpathdis = Rbecheck_paths[endpathnum].distorightline;
			
			if(show_log)
			{
				std::cout<<"right no left  "<<endpathdis<<"endpathnum "<<endpathnum<<std::endl;
			}
			
  			for(int i=(Rbecheck_paths.size()-1);i>=0;i--)
  			{
  				Rbecheck_paths[i].distorightline = endpathdis - Rbecheck_paths[i].distorightline;
  				BeCheckallpath.push_back(Rbecheck_paths[i]);
  			}
  			centerdis = Rbecheck_paths[0].distorightline/2;

  		}

	}
	else
	{
		if(!CLbecheck_paths.empty())
		{
			endpathnum = 0;

			endpathdis = CLbecheck_paths[endpathnum].distorightline;
			
			if(show_log)
			{
				std::cout<<"left no right "<<endpathdis<<"endpathnum "<<endpathnum<<std::endl;
			}
			
			for(int i=0;i<CLbecheck_paths.size();i++)
  			{
  				CLbecheck_paths[i].distorightline = endpathdis - CLbecheck_paths[i].distorightline;
  				BeCheckallpath.push_back(CLbecheck_paths[i]);
  			}
  			centerdis = CLbecheck_paths[CLbecheck_paths.size()-1].distorightline/2;
		}
		else
		{
			pathID = -1;
			if(show_log)
			{
				std::cout<<"no paths "<<std::endl;
			}
			return ;

		}

	}

	if(centerdis == 0)
	{
		pathID = -1;
		if(show_log)
		{
			std::cout<<"no paths "<<std::endl;
		}
		return ;
	}
	
	if(show_log)
	{
		std::cout<<"after after --------------------- "<<std::endl;
		for(int i=0;i<CLbecheck_paths.size();i++)
		{
			std::cout<<CLbecheck_paths[i].pathNum<<" "<<CLbecheck_paths[i].LeoRit<<" "<<CLbecheck_paths[i].distorightline<<"---- ";
		}  
		std::cout<<std::endl;  

		for(int i=0;i<Rbecheck_paths.size();i++)
		{
			std::cout<<Rbecheck_paths[i].pathNum<<" "<<Rbecheck_paths[i].LeoRit<<" "<<Rbecheck_paths[i].distorightline<<"---- ";
		}  
		std::cout<<std::endl; 

		std::cout<<"centerdis centerdis centerdis "<<centerdis<<std::endl;

		for(int j=0;j<BeCheckallpath.size();j++)
		{
			std::cout<<BeCheckallpath[j].pathNum<<" distorightline "<<BeCheckallpath[j].distorightline<<"-----";
		}
		std::cout<<std::endl;
	}
	
	float centerm_shiftpath;
	//靠中线策略方法,找出当前中线的基准距离----以距离最右端轨迹为参照
	if(Strategy_level==2)
	{
		centerline_attrib centerline;
		centerline.path_num = -1;
		centerline.belonglr = -1;
		centerline.distocen = -1;

		float distorefer;

		for(int i=0;i<LG_PathsCopy.size();i++)
		{

			CPoint2d linepoint;
			linepoint.x = LG_PathsCopy[i][1].x;
			linepoint.y = LG_PathsCopy[i][1].y;
			float referVx;
			float referVy;
			float linetoreferVx;
			float linetoreferVy;
			float angletwoline;
			referVx = partreferpath[0].x-partreferpath[partreferpath.size()-1].x;
			referVy = partreferpath[0].y-partreferpath[partreferpath.size()-1].y;
			linetoreferVx = linepoint.x - partreferpath[partreferpath.size()-1].x;
			linetoreferVy = linepoint.y - partreferpath[partreferpath.size()-1].y;
			float distoreferV;
			float dislinetoreferV;
			distoreferV = sqrt((referVx*referVx)+(referVy*referVy));
			dislinetoreferV = sqrt((linetoreferVx*linetoreferVx)+(linetoreferVy*linetoreferVy));
			angletwoline = acos((referVx*linetoreferVx+linetoreferVy*referVy)/(distoreferV*dislinetoreferV));
			
			distorefer = linepoint.Dist(partreferpath[partreferpath.size()-1])*sin(angletwoline);
			if(i==0)
			{
				centerline.path_num = i;
				centerline.belonglr = 0;
				centerline.distocen = distorefer;
			}
			else
			{
				if(distorefer < centerline.distocen)
				{
					centerline.path_num = i;
					centerline.belonglr = 0;
					centerline.distocen = distorefer;
				}
			}
		}

		for(int i=0;i<RG_PathsCopy.size();i++)
		{

			CPoint2d linepoint;
			linepoint.x = RG_PathsCopy[i][1].x;
			linepoint.y = RG_PathsCopy[i][1].y;
			float referVx;
			float referVy;
			float linetoreferVx;
			float linetoreferVy;
			float distorefer;
			float angletwoline;
			referVx = partreferpath[0].x-partreferpath[partreferpath.size()-1].x;
			referVy = partreferpath[0].y-partreferpath[partreferpath.size()-1].y;
			linetoreferVx = linepoint.x - partreferpath[partreferpath.size()-1].x;
			linetoreferVy = linepoint.y - partreferpath[partreferpath.size()-1].y;
			float distoreferV;
			float dislinetoreferV;
			distoreferV = sqrt((referVx*referVx)+(referVy*referVy));
			dislinetoreferV = sqrt((linetoreferVx*linetoreferVx)+(linetoreferVy*linetoreferVy));
			angletwoline = acos((referVx*linetoreferVx+linetoreferVy*referVy)/(distoreferV*dislinetoreferV));

			distorefer = linepoint.Dist(partreferpath[partreferpath.size()-1])*sin(angletwoline);
		
			if(distorefer < centerline.distocen)
			{
				centerline.path_num = i+1;
				centerline.belonglr = 1;
				centerline.distocen = distorefer;
			}
		}
		if(show_log)
		{
			std::cout<<"centerline "<<centerline.path_num<<" "<<centerline.belonglr<<" "<<centerline.distocen<<std::endl;
		}

		if(!Rbecheck_paths.empty())
		{
			if(show_log)
			{
				std::cout<<"R_path endpathdis "<<endpathdis<<std::endl;
			}
			
			if(centerline.belonglr == 1)
			{
				float temdistoZero; 
				temdistoZero = unitShift*centerline.path_num;
				centerline.distocen =  endpathdis - temdistoZero;
			}
			if(centerline.belonglr == 0)
			{
				float temdistoZero; 
				temdistoZero = -unitShift*centerline.path_num;
				centerline.distocen =  endpathdis - temdistoZero;
			}
		}
		else
		{
			if(!CLbecheck_paths.empty())
			{
				if(show_log)
				{
					std::cout<<"No R_path endpathdis "<<endpathdis<<std::endl;
				}
				if(centerline.belonglr == 1)
				{
					float temdistoZero; 
					temdistoZero = unitShift*centerline.path_num;
					centerline.distocen =  endpathdis - temdistoZero;
				}
				if(centerline.belonglr == 0)
				{
					float temdistoZero; 
					temdistoZero = -unitShift*centerline.path_num;
					centerline.distocen =  endpathdis - temdistoZero;
				}
			}
			else
			{
				pathID = -1;
				
				if(show_log)
				{
					std::cout<<"no paths "<<std::endl;
				}
				
				return ;
			}

		}

		if(show_log)
		{
			std::cout<<"after centerline "<<centerline.path_num<<" "<<centerline.belonglr<<" "<<centerline.distocen<<std::endl;
		}

		centerm_shiftpath = centerline.distocen-centerlineoffset; 	//得到中线距离之后再靠右多少距离
		

		if(show_log)
		{
			std::cout<<"centerm_shiftpath "<<centerm_shiftpath<<std::endl;
		}
		
		if(show_log)
		{
			std::cout<<"centerm_shiftpath "<<centerm_shiftpath<<std::endl;
		}
	}
	
	
	float standard_shiftpath = unitShiftNum*unitShift;		//直接根据最右端

	if(show_log)
	{
		std::cout<<"standard_shiftpath "<<standard_shiftpath<<std::endl;
	}
	
	//模式选择
	switch(Strategy_level)
	{
		case 1:
		{	
			if(show_log)
			{
				std::cout<<"choosing standard_shiftpath____________  "<<standard_shiftpath<<std::endl;
			}
			
			m_shiftpath = standard_shiftpath;
			break;
		}
		case 2:
		{	//中线靠右
			if(show_log)
			{
				std::cout<<"choosing centerm_shiftpath____________ "<<centerm_shiftpath<<std::endl;
			}
			
			m_shiftpath = centerm_shiftpath;
			break;
		}
		default:
		{	//默认直接获取最右端靠右
			if(show_log)
			{
				std::cout<<"choosing standard_shiftpath____________  "<<centerm_shiftpath<<std::endl;
			}
			
			m_shiftpath = standard_shiftpath;
			break;
		}
	}
	
	if(show_log)
	{
		std::cout<<"choosing m_shiftpath is  "<<m_shiftpath<<std::endl;
	}
	
	
	//根据标准路径选择
	int BeCheckallpathindex =-1;
	
	chosetraj(BeCheckallpath,m_shiftpath,BeCheckallpathindex,unitShift);
	
	if(show_log)
	{
		std::cout<<"BeCheckallpathindex "<<BeCheckallpathindex<<std::endl;
	}	
	
	//当选择是靠中线模式时，选到最右最左边的极端情况时对轨迹进行的处理
	if((Strategy_level == 2) && (BeCheckallpath.size()>=5))
    {
    	int temBeCheckallpathindex = -1;

    	int BeCheckallpathnum =-1;
    	
    	if(BeCheckallpathindex<2 || BeCheckallpathindex>(BeCheckallpath.size()-3))
    	{
    		if(BeCheckallpathindex<2)
    		{
    		 	BeCheckallpathnum = 2;
    		}
    		else
    		{
    			BeCheckallpathnum = BeCheckallpath.size()-3;
    		}
			
			if(show_log)
			{
				std::cout<<"BeCheckallpathnum "<<BeCheckallpathnum<<std::endl;
			}
    		temBeCheckallpathindex = BeCheckallpathnum;
			
			if(show_log)
			{
				std::cout<<"temBeCheckallpathindex "<<temBeCheckallpathindex<<std::endl;
    		}
			
    		int temBeCheckallpathindexup = temBeCheckallpathindex+1;
    		int temBeCheckallpathindexdown = temBeCheckallpathindex-1;
			
			if(show_log)
			{
				std::cout<<"temBeCheckallpathindexup "<<temBeCheckallpathindexup<<std::endl;
				std::cout<<"temBeCheckallpathindexdown "<<temBeCheckallpathindexdown<<std::endl;

				std::cout<<"three pathNum "<<BeCheckallpath[temBeCheckallpathindexdown].pathNum<<" "<<BeCheckallpath[temBeCheckallpathindex].pathNum<<" "<<BeCheckallpath[temBeCheckallpathindexup].pathNum<<std::endl;
			}
    		int separationdown = BeCheckallpath[temBeCheckallpathindexdown].pathNum - BeCheckallpath[temBeCheckallpathindex].pathNum;
    		int separationup = BeCheckallpath[temBeCheckallpathindexup].pathNum - BeCheckallpath[temBeCheckallpathindex].pathNum;

			if(show_log)
			{
				std::cout<<"separationdown & separationup "<<separationup<<" "<<separationdown<<std::endl;
    		}
			
			if(separationdown ==1 && separationup == 1)
    		{
    			BeCheckallpathindex = temBeCheckallpathindex;
    		}
    		if((separationdown ==-1 && separationup == 1) || (separationdown ==-1 && separationup == 1))
    		{
    			BeCheckallpathindex = temBeCheckallpathindex;
    		}
    	}
    }

	if(show_log)
	{	
     std::cout<<"after BeCheckallpathindex "<<BeCheckallpathindex<<std::endl;
	}
	
	CLnum=RG_Paths.size();
    pathID = -1;
    if(BeCheckallpath[BeCheckallpathindex].LeoRit == 1)
    {
    	 pathID = BeCheckallpath[BeCheckallpathindex].pathNum-1;
    }
    if(BeCheckallpath[BeCheckallpathindex].LeoRit == 0)
    {
    	 pathID = BeCheckallpath[BeCheckallpathindex].pathNum+CLnum;
    }
}

//获得轨迹的控制点
CPoint2d Bspline_Mtrajfinder::getgoalpoint(std::string &frame_id,std::vector <CPoint2d> subgoalpoint)
{	
	if(frame_id == "/camera_init" && flagbazersmooth)
	{
		//　用于选取子目标点的距离变量
		// distoBob = 1.2;
		distoBob = 2.2;
	}

	if(frame_id == "/camera_init" && flagbsplinesmooth)
	{
		distoBob = 0.7;
	}

	CPoint2d Getpoint;
	float distoRob=0;
	for(int i=0;i<subgoalpoint.size();i++)
	{
		distoRob = distoRob+subgoalpoint[i+1].Dist(subgoalpoint[i]);
		if(distoRob>distoBob)
		{
			Getpoint = subgoalpoint[i];
			return Getpoint;
		}

	}
	
	Getpoint = subgoalpoint[subgoalpoint.size()-1];
	return Getpoint;
}

//角度转化到180 - -180 度函数
void Bspline_Mtrajfinder::transerrorvel(float &anglerror,const float &anglevet,const float &TH)
{
	// from rob to controlpoint , Clockwise is -,Eastern is +
	float maxangle,minangle;
    if(anglevet>=0)
    {
		maxangle=anglevet;
		minangle=anglevet-180;
	    if(TH <maxangle && TH>=minangle)
		{
		 anglerror=anglerror;
		}
		else
		{
		anglerror=-1*anglerror;
		}
	}
	if(anglevet<0)
	{
		maxangle=anglevet+180;
		minangle=anglevet;
		if(TH<maxangle && TH>=minangle)
		{
		anglerror=-1*anglerror;
		}
		else
		{
		anglerror=anglerror;
		}
	}
}

//根据选取的控制点和机器人位置，角度计算需要发出的线速度和角速度
void Bspline_Mtrajfinder::ComputeCmd(std::string &frame_id,
										const CPoint2d &controlpoint,
										Cmd_level &Control_Cmd)
{
	//
	
	
	float anglevet1,anglevet2,anglerror,anglerror1,distoconpoint,anglerrorAbs;
	// 用于计算转弯半径、角度约束的变量
	float l_2, R, angleOrient, angleOrientAbs, direct;

	std::cout<<"controlpoint is "<<controlpoint.x<<" "<<controlpoint.y << " " << controlpoint.pointangle << std::endl;
	
	float vetangleX,vetangleY;
	float TH;
	
	if(frame_id=="/rslidar")
	{
		vetangleX = controlpoint.x - 0.0;
		vetangleY = controlpoint.y - 0.0;
		TH = 0.0;
	}
	
	if(frame_id=="/camera_init")
	{
		vetangleX = controlpoint.x - Rob.x;
		vetangleY = controlpoint.y - Rob.y;
		TH = Rob.pointangle;
	}

	// atan(y/x) -pi/2到pi/2
	anglevet1 = toDegree(atan2(vetangleY,vetangleX));
	
	std::cout<<"------------------------get goal anglevet1 "<<std::endl;
	std::cout<<anglevet1<<std::endl;
	
	// acos 0-pi
	anglerror=acos(cos(TH)*cos(toRadian(anglevet1))+sin(TH)*sin(toRadian(anglevet1)));
	// toDegree -180  180
  	anglerror=toDegree(anglerror);
  	anglerrorAbs = anglerror;
  	std::cout << anglerrorAbs << "                  " << anglerror << std::endl;
    TH=toDegree(TH);
	
	// 角转到180 -180,输出的error考虑了方向
	transerrorvel(anglerror,anglevet1,TH);
	
	std::cout<<"now rob TH "<<TH<<std::endl;
	std::cout<<"anglerror anglerror anglerror "<<anglerror<<std::endl;


	// 计算转弯半径
	angleOrient = toDegree(Rob.pointangle - controlpoint.pointangle);
	angleOrientAbs = abs(angleOrient);
	l_2 = vetangleX * vetangleX + vetangleY * vetangleY;
	R = sqrt(l_2 / (2 * (1 - cos(anglerrorAbs))));

	// 根据目标点与当前方向夹角 确定转弯方向
	if (anglerror >= 0) {
		direct = 1.0;
	}
	else {
		direct = -1.0;
	}
	// 根据目标点与当前方向夹角  转弯半径  方向变化角  确定速度
	if (anglerrorAbs < 5) {
		Control_Cmd.linear = 0.3;
		Control_Cmd.twist = 0.0 * direct;
	}
	else if (anglerrorAbs >= 80) {
		// 当转弯角度很大时，此处阈值为80，直接以最小半径转弯
		Control_Cmd.linear = 0.16;
		Control_Cmd.twist = 0.1 * direct;
	}
	else {
		// 限制转弯半径
		R = getLimitedRadius(R);

		if (angleOrient * anglerror >= 0) {
			if (abs(angleOrientAbs - anglerrorAbs) <= 10) {
				Control_Cmd.linear = 0.25;
				Control_Cmd.twist = 0.25 / R * direct;
			}
			else if (angleOrientAbs < anglerrorAbs - 10) {
				// 方向变化较小，所以以1/2处为目标位置，为回正预留距离
				R = getLimitedRadius(R * 0.707);
				Control_Cmd.linear = 0.25;
				Control_Cmd.twist = 0.25 / R * direct;
			}
			else if (angleOrientAbs > anglerrorAbs + 10) {
				// 方向变化较大，略微缩小转弯半径，以增大转过的角度
				R = getLimitedRadius(R * 0.866);
				Control_Cmd.linear = 0.25;
				Control_Cmd.twist = 0.25 / R * direct;
			}
		}
		else if (angleOrient * anglerror < 0) {
			if(angleOrientAbs + anglerrorAbs <= 10) {
				Control_Cmd.linear = 0.25;
				Control_Cmd.twist = 0.25 / R * direct;
			}
			else if (angleOrientAbs + anglerrorAbs > 10) {
				// 方向不同，相差较大时，以1/2处为目标，为回正预留距离，同上第二种情况
				R = getLimitedRadius(R * 0.707);
				Control_Cmd.linear = 0.25;
				Control_Cmd.twist = 0.25 / R * direct;
			}
		}
	}

	
	//根据角度差来划分速度等级
	// int WandVlevel;
	// WandVlevel = anglerrorAbs/20;
	// if(show_log)
	// {
	// 	 std::cout<<"anglerrorAbs "<<anglerrorAbs<<std::endl;
	// 	 std::cout<<"WandVlevel "<<WandVlevel<<std::endl;
	// }
		
	// if(anglerrorAbs<5)
	// {
	 		
	// 	Control_Cmd.twist = 0.0;
	// 	Control_Cmd.linear = 0.7;
	// 	std::cout<<"line line "<<std::endl;
 //    }
	// else
	// {
	//  	switch(WandVlevel)
	//  	{
	//  		case(0):
	//  		{
	//  			if(anglerrorAbs>=5 && anglerrorAbs<=10)
 // 				{
 // 					if(anglerror>=0)
 // 					{
 						
 // 						Control_Cmd.twist = 0.0;
	// 					Control_Cmd.linear = 0.7;
 // 						//std::cout<<"anglerror>=0 anglerrorAbs>=5 && anglerrorAbs<=10 "<<std::endl;
 // 						break;
 // 					}
 // 					else
 // 					{
 // 						Control_Cmd.twist = 0.0;
	// 					Control_Cmd.linear = 0.7;
 // 						//std::cout<<"else anglerror>=0 anglerrorAbs>=5 && anglerrorAbs<=10 "<<std::endl;
 // 						break;
 // 					}
 // 				}
 // 				else
 // 				{
 // 					if(anglerror>=0)
 // 					{
	// 					Control_Cmd.twist = 0.1745;
	// 					Control_Cmd.linear = 0.6;
 // 						//std::cout<<"0.6 30(10-20) ++  "<<std::endl;
 // 						break;
 // 					}
 // 					else
 // 					{
 // 						Control_Cmd.twist = -0.1745;
	// 					Control_Cmd.linear = 0.6;
 // 						//std::cout<<"0.6 30(10-20) --  "<<std::endl;
 // 						break;
 // 					}
 // 				}
 // 			}//0.6 30(0-20)
				
 // 			case(1):
 // 			{
 // 				if(anglerror>=0)
 // 				{
 // 					Control_Cmd.twist = 0.2618;
	// 				Control_Cmd.linear = 0.5;
 // 					//std::cout<<"//0.4 40 (20-40)++  "<<std::endl;
 // 					break;
 // 				}
 // 				else
 // 				{
	// 				Control_Cmd.twist = -0.2618;
	// 				Control_Cmd.linear = 0.5;
 // 					//std::cout<<"//0.4 40 (20-40) -- "<<std::endl;
 // 					break;
 // 				}
 // 			}//0.4 40 (20-40)

 // 			case(2):
 // 			{
 // 				if(anglerror>=0)
 // 				{
 // 					Control_Cmd.twist = 0.3491;
	// 				Control_Cmd.linear = 0.4;
 // 					//std::cout<<"//0.4 60(40-60) ++ "<<std::endl;
 // 					break;
 // 				}
 // 				else
 // 				{
 // 					Control_Cmd.twist = -0.3491;
	// 				Control_Cmd.linear = 0.4;
 // 					//std::cout<<"//0.4 60(40-60) -- "<<std::endl;
 // 					break;
 // 				}
 // 			}//0.4 60(40-60)

 // 			case(3):
 // 			{
 // 				if(anglerror>=0)
 // 				{
	// 				Control_Cmd.twist = 0.4363;
	// 				Control_Cmd.linear = 0.2;
 // 					//std::cout<<"//0.2 90(60-80) ++ "<<std::endl;
 // 					break;
 // 				}
 // 				else
 // 				{
	// 				Control_Cmd.twist = -0.4363;
	// 				Control_Cmd.linear = 0.2;
 // 					//std::cout<<"//0.2 90(60-80) -- "<<std::endl;
 // 					break;
 // 				}
 // 			}//0.2 90(60-80)

 // 			default:
 // 			{
 // 				if(anglerror>=0)
 // 				{
 // 					Control_Cmd.twist = 0.5236;
	// 				Control_Cmd.linear = 0.15;
 // 					//std::cout<<"++ 80+ "<<std::endl;
 // 					break;
 // 				}
 // 				else
 // 				{
 // 					Control_Cmd.twist = -0.5236;
	// 				Control_Cmd.linear = 0.15;
 // 					//std::cout<<"--- 80-- "<<std::endl;
 // 					break;
 // 				}
 // 			}
 // 		}
 // 	}



}

void Bspline_Mtrajfinder::CoverRCmd(std::vector<CPoint2d> subgoalpoint,Cmd_level &Control_Cmd)
{
	CPoint2d sampoint = subgoalpoint[3];
			
	float th = atan2((subgoalpoint[3].y - subgoalpoint[2].y),(subgoalpoint[3].x - subgoalpoint[2].x));
	
	std::cout<<"th "<<th<<std::endl;

	float vecy = sin(th);
	float vecx = cos(th);

	float dis = sqrt((vecx * vecx) + (vecy * vecy));

	float kx = -1 * vecy / (dis);
	float ky = vecx / (dis);

	float kth = atan2(ky,kx);


	float b = sampoint.y - tan(kth) * sampoint.x;

	float r =  b;

	if(r>0)
	{

		std::cout<<"r "<<r<<std::endl;
	}
	else
	{
		std::cout<<"r "<<(-1 * r)<<std::endl;
	}

	//转弯半径作为选择控制，乘以一百倍好作为选择
	int level = sqrt(r * r) * 100;
	std::cout<<"level "<<level<<std::endl;

	if(level>=100)
	{
		Control_Cmd.linear = 0.8;
		Control_Cmd.twist = Control_Cmd.linear/r;
	}

	if(level<100 && level>=50)
	{
		Control_Cmd.linear = 0.6;
		Control_Cmd.twist = Control_Cmd.linear/r;
	}


	if(level<50 && level>=10)
	{
		Control_Cmd.linear = 0.6;
		Control_Cmd.twist = Control_Cmd.linear/r;
	}


	if(level<10 && level>=5)
	{
		Control_Cmd.linear = 0.5;
		Control_Cmd.twist = Control_Cmd.linear/r;
	}

	if(level<5 && level>=4)
	{
		Control_Cmd.linear = 0.4;
		Control_Cmd.twist = Control_Cmd.linear/r;
	}

	if(level<4 && level>1)
	{
		Control_Cmd.linear = 0.15;
		Control_Cmd.twist = Control_Cmd.linear/r;
	}

	if(level<=1)
	{
		Control_Cmd.linear = 0;
		Control_Cmd.twist = 1.57;
	}
}
