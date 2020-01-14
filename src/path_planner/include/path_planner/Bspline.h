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
	std::vector<CPoint2d> BSplinePoints; //�����b�����ϵĵ�
	std::vector<double> NodeVector;

	bool m_isFreeBoundary; 
	CPoint2d T0; //��ʧ
	CPoint2d Tn;

	void GetNodeVector(); //���������ֵ����������ģ�ͼ���ڵ�ʸ��
	void GetControlPnts(); //��׷�Ϸ�����ֵ�������Ƶ㣨���������ߣ�
	void GetContPntsOfClosedBS(); //��׷�Ϸ�����ֵ�������Ƶ㣨�������ߣ�
	void GetBSplinePnts(); //����GetdeBoorValue()
	CPoint2d GetdeBoorValue(int i,double t);


};
#endif 
