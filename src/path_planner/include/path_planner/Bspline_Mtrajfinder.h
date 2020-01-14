#ifndef Bspline_Mtrajfinder_H
#define Bspline_Mtrajfinder_H
#include <vector>
#include "path_planner/point.h"
#include "nav_msgs/OccupancyGrid.h"

//轨迹属性的结构体，存储轨迹位置信息，编号
//是机器人方向的左边还是右边，与最右端轨迹距离
struct pro_path
{
	std::vector <CPoint2d> Ex_path;
	int pathNum;     // left or right 's num
	int LeoRit;     //represent left or right 0 1
	float distorightline; //dis to most right line
};

//用于靠中线策略中存储靠中线轨迹一些信息的结构体
struct centerline_attrib
{
	int path_num;   // path num
	int belonglr;   // represent left or right 0 1
	float distocen; // less dis to referpath this path
};

struct Cmd_level
{
	float twist;
	float linear;
};


class Bspline_Mtrajfinder
{
	public:
	
	bool flagbsplinesmooth;
	bool flagbazersmooth;
	
	bool Subgoalstart;
	CPoint2d Rob;
	CPoint2d lastRob;
	CPoint2d subGoal;
	
	float m_roadWidth;
	float extra_roadWidth;
	float roadways;
	float roadwithnum;
	
	float judgeOblen;      //判断障碍的轨迹长度
	float unitShiftNum;
	float Ob_thread;
	float distoBob;
	float angletoEnd;
	float intervaltrajNum;
	float centerlineoffset;

	float Mapreluton;				//µØÍŒ·Ö±æÂÊ
	float Map_originPx;             //µØÍŒÆðÊŒÈ«ŸÖ×ø±ê
	float Map_originPy;				//µØÍŒÆðÊŒÈ«ŸÖ×ø±ê
	int Mapheight;					//µØÍŒžß¶ÈºÍ¿í¶È
	int Mapwidth;
	

	
	int refer_pathindex;
	int pathID;
	int CLnum;
	int Strategy_level;
	int sampnum;
	
	bool show_log; 
	bool codingstart;
	
	std::vector<CPoint2d> m_refer_path;
	std::vector<std::vector<CPoint2d> > LG_Paths;
	std::vector<std::vector<CPoint2d> > LG_PathsCopy;
	std::vector<std::vector<CPoint2d> > RG_Paths;
	std::vector<std::vector<CPoint2d> > RG_PathsCopy;
	std::vector<std::vector<CPoint2d> > LGJudgeOb_paths;
	std::vector<std::vector<CPoint2d> > RGJudgeOb_paths;
	
	//经过障碍物选择之后的左右轨迹存储向量
	std::vector<pro_path> CLbecheck_paths;
	std::vector<pro_path> Rbecheck_paths;
	
	std::vector<int> MapObindex;    //障碍物栅格属性1是障碍-1不是
	
	//最终截取的先验路径的一段
	std::vector<CPoint2d> partreferpath;
	
	CPoint2d Goalpoint;
	
	
	Bspline_Mtrajfinder();
	Bspline_Mtrajfinder(std::vector<CPoint2d> key_point);
	~Bspline_Mtrajfinder();
	
	void set_navpoint(std::vector<CPoint2d> key_point);
	
	void Getsubgoal(std::string &frame_id,CPoint2d &subgoal);
	bool Rechgoal();
	void ResizeGlobalpath();
	void Generatraj(std::string &frame_id);
	bool Checkpath(std::vector<CPoint2d> &potential_paths,
				   std::vector<int> &temMapObindex);
				   
	void Setmapindex(const nav_msgs::OccupancyGrid &msg);
	
	void chosetraj(std::vector<pro_path> &BeCheckallpath,
					const float &m_shiftpath,int &BeCheckallpathindex,const float &unitShift);
	
	bool chosepath(std::vector<pro_path> &BeCheckallpath,const float &dis,int &num);
					
	CPoint2d getgoalpoint(std::string &frame_id,std::vector <CPoint2d> subgoalpoint);
	
	void ComputeCmd(std::string &frame_id,
					const CPoint2d &controlpoint,
					Cmd_level &Control_Cmd);
	void CoverRCmd(std::vector <CPoint2d> subgoalpoint,Cmd_level &Control_Cmd);
	
	void transerrorvel(float &anglerror,const float &anglevet,const float &TH);

	void ChangePath(std::vector<CPoint2d> &oringinPath,std::vector<CPoint2d> &nowPath, float rightShift);
	
};
#endif

