#ifndef point_H
#define point_H

class CPoint2d
{
public:
	float x;	// 2d点的x坐标
	float y;	// 2d点的y坐标
	float pointangle;
	

public:
	// 构造函数
	CPoint2d(void);
	CPoint2d(float x,float y);
	~CPoint2d(void);
	// 复制构造函数
	CPoint2d(const CPoint2d& tmpP);

	// 重载运算符为成员函数
	CPoint2d & operator += ( const CPoint2d & p);
	CPoint2d & operator -= ( const CPoint2d & p);
	CPoint2d & operator *= ( float s );
	CPoint2d & operator /= ( float s );
	// 重载运算符为友元函数

	friend bool operator == (const CPoint2d &p1,const CPoint2d &p2);
	friend bool operator != (const CPoint2d &p1,const CPoint2d &p2);	
	friend CPoint2d operator + (const CPoint2d &p1 ,const CPoint2d &p2);
	friend CPoint2d operator - (const CPoint2d &p1 ,const CPoint2d &p2);
	friend CPoint2d operator * ( const CPoint2d &p , float s );
	friend CPoint2d operator / (const CPoint2d &p ,float num);

	// 点到原点的距离
	//double Dist(void);
	// 点到原点的距离
	float Dist(const CPoint2d &p);
};

#endif 