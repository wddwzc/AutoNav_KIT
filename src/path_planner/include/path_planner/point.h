#ifndef point_H
#define point_H

class CPoint2d
{
public:
	float x;	// 2d���x����
	float y;	// 2d���y����
	float pointangle;
	

public:
	// ���캯��
	CPoint2d(void);
	CPoint2d(float x,float y);
	~CPoint2d(void);
	// ���ƹ��캯��
	CPoint2d(const CPoint2d& tmpP);

	// ���������Ϊ��Ա����
	CPoint2d & operator += ( const CPoint2d & p);
	CPoint2d & operator -= ( const CPoint2d & p);
	CPoint2d & operator *= ( float s );
	CPoint2d & operator /= ( float s );
	// ���������Ϊ��Ԫ����

	friend bool operator == (const CPoint2d &p1,const CPoint2d &p2);
	friend bool operator != (const CPoint2d &p1,const CPoint2d &p2);	
	friend CPoint2d operator + (const CPoint2d &p1 ,const CPoint2d &p2);
	friend CPoint2d operator - (const CPoint2d &p1 ,const CPoint2d &p2);
	friend CPoint2d operator * ( const CPoint2d &p , float s );
	friend CPoint2d operator / (const CPoint2d &p ,float num);

	// �㵽ԭ��ľ���
	//double Dist(void);
	// �㵽ԭ��ľ���
	float Dist(const CPoint2d &p);
};

#endif 