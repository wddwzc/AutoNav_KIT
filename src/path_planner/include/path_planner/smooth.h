#ifndef smooth_H
#define smooth_H
#include <vector>
#include "path_planner/point.h"	// ������CPoint2d������ṹ

class CBSpline
{
public:
	/*!
	* \fn CBSpline
	* \brief Ĭ�Ϲ��캯�������������ֱ�Ϊ��ֵ��������power������ֵ�㼯��SomeArray
	*/
	CBSpline(int power, std::vector<CPoint2d>& SomeArray);
	
	

	/*!
	* \fn ~CBSpline
	* \brief Ĭ����������
	*/
	~CBSpline(void);

	/*!
	* \fn GetBSplinePoint
	* \brief �ͻ��˻�ò�ֵƽ����ĵ㼯��
	*/
	std::vector<CPoint2d> & GetBSplinePoint();

private:

	/*!
	* \fn Alpha	dComputing	GetSpotVector
	* \brief �ͻ������迼�ǵ��ڲ�����
	*/
	double Alpha(int index, double U, int step);
	void dComputing(std::vector<CPoint2d> &PtAry, int i, double u);
	std::vector<double> GetSpotVector( int PointCount,int Power );

private:

	std::vector<double> SpotVector;
	int PointCount;
	int Power;						// ��ֵ�����Ĵ���
	int StepofDeBoor;
	std::vector<CPoint2d> PointArray;	// �洢����ֵ�ĵ㼯
	std::vector<CPoint2d> PointLine;		// ���շ������û��ĵ㼯
};


class CBezier
{  
public:

	/*!
	* \fn CBezier
	* \brief Ĭ�Ϲ��캯��
	*/
	CBezier();

	/*!
	* \fn ~CBezier
	* \brief Ĭ����������
	*/
	~CBezier();

	/*!
	* \fn Bezier
	* \brief �ͻ��˻�ò�ֵƽ����ĵ㼯��NewPoints������ֵ�㼯ΪOriPoints
	*/
	void Bezier(std::vector<CPoint2d> &OriPoints,std::vector<CPoint2d> &NewPoints);
};


#endif  
