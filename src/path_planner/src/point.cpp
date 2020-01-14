#include "path_planner/point.h"
#include <math.h>

//////////////////////////////////////////////////////////////////////////
/////////// 默认构造函数 ///////////
CPoint2d::CPoint2d(void)
:x(0.0),y(0.0),pointangle(0.0)
{
}

CPoint2d::CPoint2d(float x,float y)
{
	this->x = x;
	this->y = y;
}

// 析构函数
CPoint2d::~CPoint2d(void)
{
}

// 复制构造函数
CPoint2d::CPoint2d(const CPoint2d& tmpP)
{
	this->x = tmpP.x;
	this->y = tmpP.y;
	this->pointangle = tmpP.pointangle;	
}

//////////////////////////////////////////////////////////////////////////
/////////// 重载运算符为成员函数 ///////////
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

// 重载运算符为友元函数
CPoint2d operator + (const CPoint2d &p1 ,const CPoint2d &p2)	// 相加
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
CPoint2d operator / (const CPoint2d &p ,float num)			// 除以一个整数
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

bool operator == (const CPoint2d &p1,const CPoint2d &p2)	// 相等
{
	if(p1.x==p2.x && p1.y==p2.y )
		return true;
	else 
		return false;
}
bool operator != (const CPoint2d &p1,const CPoint2d &p2)	// 不等
{
	if(p1.x==p2.x && p1.y==p2.y)
		return false;
	else 
		return true;
}

//////////////////////////////////////////////////////////////////////////
// 点到原点的距离
//double CPoint2d::Dist(void)
//{
//	return sqrt(x*x+y*y);
//}

// 点到另一个点的距离
float CPoint2d::Dist(const CPoint2d &p)
{
	return sqrt((x-p.x)*(x-p.x)+(y-p.y)*(y-p.y));
}

