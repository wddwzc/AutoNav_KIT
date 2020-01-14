#ifndef smooth_H
#define smooth_H
#include <vector>
#include "path_planner/point.h"	// 运用了CPoint2d基础点结构

class CBSpline
{
public:
	/*!
	* \fn CBSpline
	* \brief 默认构造函数，两个参数分别为差值函数次数power、待差值点集合SomeArray
	*/
	CBSpline(int power, std::vector<CPoint2d>& SomeArray);
	
	

	/*!
	* \fn ~CBSpline
	* \brief 默认析构函数
	*/
	~CBSpline(void);

	/*!
	* \fn GetBSplinePoint
	* \brief 客户端获得差值平滑后的点集合
	*/
	std::vector<CPoint2d> & GetBSplinePoint();

private:

	/*!
	* \fn Alpha	dComputing	GetSpotVector
	* \brief 客户端无需考虑的内部函数
	*/
	double Alpha(int index, double U, int step);
	void dComputing(std::vector<CPoint2d> &PtAry, int i, double u);
	std::vector<double> GetSpotVector( int PointCount,int Power );

private:

	std::vector<double> SpotVector;
	int PointCount;
	int Power;						// 插值函数的次数
	int StepofDeBoor;
	std::vector<CPoint2d> PointArray;	// 存储待差值的点集
	std::vector<CPoint2d> PointLine;		// 最终反馈给用户的点集
};


class CBezier
{  
public:

	/*!
	* \fn CBezier
	* \brief 默认构造函数
	*/
	CBezier();

	/*!
	* \fn ~CBezier
	* \brief 默认析构函数
	*/
	~CBezier();

	/*!
	* \fn Bezier
	* \brief 客户端获得差值平滑后的点集合NewPoints，待差值点集为OriPoints
	*/
	void Bezier(std::vector<CPoint2d> &OriPoints,std::vector<CPoint2d> &NewPoints);
};


#endif  
