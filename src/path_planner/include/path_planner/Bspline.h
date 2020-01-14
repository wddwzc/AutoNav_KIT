#ifndef Bspline_H
#define Bspline_H
#include "path_planner/point.h"	
#include <vector>
#define DRAW_BSPLINE_STEP  100
class Bspline
{
public:
	Bspline();
	~Bspline();
	std::vector<CPoint2d> ShapePoints;
	std::vector<CPoint2d> ControlPoints;
	std::vector<CPoint2d> BSplinePoints; //计算出b样条上的点
	std::vector<double> NodeVector;

	bool m_isFreeBoundary; 
	CPoint2d T0; //切失
	CPoint2d Tn;

	void GetNodeVector(); //由输入的型值点利用向心模型计算节点矢量
	void GetControlPnts(); //用追赶法由型值点计算控制点（非周期曲线）
	void GetContPntsOfClosedBS(); //用追赶法由型值点计算控制点（周期曲线）
	void GetBSplinePnts(); //调用GetdeBoorValue()
	CPoint2d GetdeBoorValue(int i,double t);


};
#endif 
