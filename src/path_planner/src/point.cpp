#include "path_planner/point.h"
#include <math.h>

//////////////////////////////////////////////////////////////////////////
/////////// Ĭ�Ϲ��캯�� ///////////
CPoint2d::CPoint2d(void)
:x(0.0),y(0.0),pointangle(0.0)
{
}

CPoint2d::CPoint2d(float x,float y)
{
	this->x = x;
	this->y = y;
}

// ��������
CPoint2d::~CPoint2d(void)
{
}

// ���ƹ��캯��
CPoint2d::CPoint2d(const CPoint2d& tmpP)
{
	this->x = tmpP.x;
	this->y = tmpP.y;
	this->pointangle = tmpP.pointangle;	
}

//////////////////////////////////////////////////////////////////////////
/////////// ���������Ϊ��Ա���� ///////////
CPoint2d & CPoint2d::operator += ( const CPoint2d & p)
{
	this->x += p.x;
	this->y += p.y;
	return *this;
}
CPoint2d & CPoint2d::operator -= (const CPoint2d & p)
{
	this->x -= p.x;
	this->y -= p.y;
	return *this;
}
CPoint2d & CPoint2d::operator *= (float s )
{
	this->x *= s;
	this->y *= s;
	return *this;
}
CPoint2d & CPoint2d::operator /= (float s )
{
	this->x /= s;
	this->y /= s;
	return *this;
}

// ���������Ϊ��Ԫ����
CPoint2d operator + (const CPoint2d &p1 ,const CPoint2d &p2)	// ���
{	
	CPoint2d po;
	po.x = p1.x + p2.x;
	po.y = p1.y + p2.y;
	return po;
}
CPoint2d operator - (const CPoint2d &p1 ,const CPoint2d &p2)	// 
{	
	CPoint2d po;
	po.x = p1.x - p2.x;
	po.y = p1.y - p2.y;
	return po;
}
CPoint2d operator * ( const CPoint2d &p , float s )
{
	CPoint2d po;
	po.x = p.x * s;
	po.y = p.y * s;
	return po;
}
double operator * ( const CPoint2d &p1 ,const CPoint2d &p2 )
{
	return ( p1.x*p2.x + p1.y*p2.y );
}
CPoint2d operator / (const CPoint2d &p ,float num)			// ����һ������
{		
	if(num != 0)
	{   
		CPoint2d po;
		po.x = p.x / num;
		po.y = p.y / num;
		return po;
	}
	else 
	{
		return CPoint2d(0,0);
	}
}

bool operator == (const CPoint2d &p1,const CPoint2d &p2)	// ���
{
	if(p1.x==p2.x && p1.y==p2.y )
		return true;
	else 
		return false;
}
bool operator != (const CPoint2d &p1,const CPoint2d &p2)	// ����
{
	if(p1.x==p2.x && p1.y==p2.y)
		return false;
	else 
		return true;
}

//////////////////////////////////////////////////////////////////////////
// �㵽ԭ��ľ���
//double CPoint2d::Dist(void)
//{
//	return sqrt(x*x+y*y);
//}

// �㵽��һ����ľ���
float CPoint2d::Dist(const CPoint2d &p)
{
	return sqrt((x-p.x)*(x-p.x)+(y-p.y)*(y-p.y));
}

