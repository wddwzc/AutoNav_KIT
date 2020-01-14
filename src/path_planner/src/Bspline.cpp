#include "path_planner/Bspline.h"	
#include "math.h"
#include <iostream>


Bspline::Bspline()
{
	m_isFreeBoundary = true; 

	/*T0.x = 20.0; //切失
	T0.y = 20.0;
	Tn.x = 20.0;
	Tn.y = 20.0;*/

	T0.x = 1.0; //切失
	T0.y = 1.0;
	Tn.x = 1.0;
	Tn.y = 1.0;

}

Bspline::~Bspline()
{


}

void Bspline::GetNodeVector() //由输入的型值点利用向心模型计算节点矢量
{

	// for(int j=0;j<ShapePoints.size();j++)
	// {
		
	// 	std::cout<<"ShapePoints "<<ShapePoints[j].x<<" "<<ShapePoints[j].y<<std::endl;
	// }


	int num = ShapePoints.size();

	double* distance = new double[num-1];
	double totalDistance = 0; //总弦长
	for (int i = 0; i < num -1; i++)
	{
		double x1 = ShapePoints[i].x;
		double x2 = ShapePoints[i+1].x;
		double y1 = ShapePoints[i].y;
		double y2 = ShapePoints[i+1].y;
		distance[i] = sqrt(sqrt(pow((x1-x2),2)+pow((y1-y2),2)));
		totalDistance += distance[i];
	}

	for (int i = 0; i < 4; i++)
	{
		NodeVector.push_back(0.0);
	}
	
	double temp; 
	for (int i = 4; i < num+2; i++)
	{
		temp = NodeVector[i-1] + distance[i-4]/totalDistance;
		NodeVector.push_back(temp);
	}
	
	for (int i = num+2; i < num+6; i++)
	{
		NodeVector.push_back(1.0);
	}

	delete distance;
	return;
}

void Bspline::GetControlPnts()
{
		int num = ShapePoints.size();

		std::vector<double> alpha;
		std::vector<double> beta;
		std::vector<double> gama;

		CPoint2d point(0.0,0.0);
		for (int i = 0; i < num+2; i++)
		{
			ControlPoints.push_back(point);
		}

		for (int i = 0; i < num; i++) //计算alpha,beta,gama
		{
			double a = NodeVector[i+4] - NodeVector[i+3];
			double b = NodeVector[i+4] - NodeVector[i+2];
			double c = NodeVector[i+4] - NodeVector[i+1];
			double d = NodeVector[i+3] - NodeVector[i+2];
			double e = NodeVector[i+5] - NodeVector[i+2];

			double alpha_i = (a*a)/(b*c);
			double gama_i = (d*d)/(e*b);
			double beta_i = 1 - alpha_i - gama_i;

			alpha.push_back(alpha_i);
			beta.push_back(beta_i);
			gama.push_back(gama_i);
		}

		double alpha_0,beta_0,gama_0;
		double alpha_n,beta_n,gama_n;
		std::vector<double> a;
		std::vector<double> b;
		std::vector<double> c;
		std::vector<CPoint2d> d;
		std::vector<double> pVector;
		std::vector<double> qVector;

		//自由边界条件下
	if (m_isFreeBoundary) 
	{
		alpha_0 = 6.0;
		beta_0 = 6.0*(2.0*NodeVector[0]-NodeVector[4]-NodeVector[5])/(NodeVector[5]-NodeVector[0]);
		gama_0 = 6.0*(NodeVector[4]-NodeVector[0])/(NodeVector[5]-NodeVector[0]);
		beta_n = 6.0*(NodeVector[num+1]+NodeVector[num]-2.0*NodeVector[num+4])/(NodeVector[num+4]-NodeVector[num]);
		alpha_n = 6.0*(NodeVector[num+4]-NodeVector[num+1])/(NodeVector[num+4]-NodeVector[num]);
		gama_n = 6.0;

		//初始化a，b，c，d
		a.push_back(0.0);
		b.push_back(1.0);
		c.push_back(0.0);
		d.push_back(ShapePoints[0]);
		a.push_back(alpha_0);
		b.push_back(beta_0);
		c.push_back(gama_0);
		CPoint2d Point(0.0,0.0);
		d.push_back(Point);
		
		for (int i = 1; i < num-1; i++)
		{
			a.push_back(alpha[i]);
			b.push_back(beta[i]);
			c.push_back(gama[i]);
			d.push_back(ShapePoints[i]);
		}
		
		a.push_back(alpha_n);
		b.push_back(beta_n);
		c.push_back(gama_n);
		d.push_back(Point);
		a.push_back(0.0);
		b.push_back(1.0);
		c.push_back(0.0);
		d.push_back(ShapePoints[num-1]);
	}

		//切失边界条件下
	if (!m_isFreeBoundary ) 
	{
		alpha_0 = -1;
		beta_0 = 1;
		beta_n = -1;
		gama_n = 1;
		gama_0 = alpha_n = 0;

		//归一化切矢量
		double temp = T0.x + T0.y; 
		CPoint2d normT0(T0.x/temp, T0.y/temp);
		temp = Tn.x + Tn.y;
		CPoint2d normTn(Tn.x/temp, Tn.y/temp);

		a.push_back(0.0);
		b.push_back(1.0);
		c.push_back(0.0);
		d.push_back(ShapePoints[0]);
		a.push_back(alpha_0);
		b.push_back(beta_0);
		c.push_back(0.0);
		d.push_back(normT0);

		for (int i = 1; i < num-1; i++)
		{
			a.push_back(alpha[i]);
			b.push_back(beta[i]);
			c.push_back(gama[i]);
			d.push_back(ShapePoints[i]);
		}

		a.push_back(0.0);
		b.push_back(beta_n);
		c.push_back(gama_n);
		d.push_back(normTn);
		a.push_back(0.0);
		b.push_back(1.0);
		c.push_back(0.0);
		d.push_back(ShapePoints[num-1]);
	}

	//根据重复度修改三对角矩阵
	double m21, m23, m22;
	for(int i = 1; i <= num-2; i++)
	{		
		if ((i >= 3) && (ShapePoints[i] == ShapePoints[i-2])) 
		{			
			m21 = 0.5;
			m23 = 0.0;
			m22 = 0.5;

			a[i+1] =  m21;
			b[i+1] = m22;
			c[i+1] = m23;
		}
		else if ((ShapePoints[i] == ShapePoints[i+1]) && (ShapePoints[i] == ShapePoints[i-1])) 
		{			
			m21 = 0.0;
			m23 = 0.0;
			m22 = 1.0;
			a[i+1] = m21;
			b[i+1] = m22;
			c[i+1] = m23;
		}
		else if ((i <= num-4) && (ShapePoints[i] == ShapePoints[i+2])) 
		{
			m21 = 0.0;
			m23 = 0.5;
			m22 = 0.5;
			a[i+1] = m21;
			b[i+1] = m22;
			c[i+1] =  m23;
		}
		else if ((i <= num-3) && (ShapePoints[i] == ShapePoints[i+1])
			&& (ShapePoints[i] != ShapePoints[i-1])) 
		{			
			m21 = 0.0;
			m23 = 0.5;
			m22 = 0.5;
			a[i+1] = m21;
			b[i+1] = m22;
			c[i+1] = m23;
		}
	}

	//计算出p矢量和q矢量 
	pVector.push_back(b[0]);
	int n = num+1;
	qVector.push_back(c[0]/pVector[0]);

	double temp;
	for (int i=1; i< n; i++)
	{
		temp = b[i] - a[i]*qVector[i-1];
		pVector.push_back(temp);
		qVector.push_back(c[i]/pVector[i]);
	}
	temp = b[n] - a[n]*qVector[n-1];
	pVector.push_back(temp);

	//计算z矢量
	std::vector<double> zVectorX;
	std::vector<double> zVectorY;
	
	double tempX, tempY;
	tempX = d[0].x / pVector[0];
	tempY = d[0].y / pVector[0];
	zVectorX.push_back(tempX);
	zVectorY.push_back(tempY);

	for (int i = 1; i <= n; i ++)
	{
		tempX = (d[i].x - a[i]*zVectorX[i-1])/pVector[i];
		zVectorX.push_back(tempX);
		tempY = (d[i].y - a[i]*zVectorY[i-1])/pVector[i];
		zVectorY.push_back(tempY);
	}

		//计算出控制点
	//point.x = (long)zVectorX[n];
	//point.y = (long)zVectorY[n];
	
	point.x =  zVectorX[n];
	point.y =  zVectorY[n];
	ControlPoints[n] = point;

	for (int i = n-1; i >= 0; i--)
	{
		//point.x = (long)(zVectorX[i] - qVector[i]*ControlPoints[i+1].x);
		//point.y = (long)(zVectorY[i] - qVector[i]*ControlPoints[i+1].y);

		point.x = (zVectorX[i] - qVector[i]*ControlPoints[i+1].x);
		point.y = (zVectorY[i] - qVector[i]*ControlPoints[i+1].y);
		ControlPoints[i] = point;
	}
}


CPoint2d Bspline::GetdeBoorValue(int i,double t)
{
	int k = 3; //曲线的次数
	
	CPoint2d* points = new CPoint2d[k+1]; 

	for (int j = i-k, index = 0; j <= i; j++)
	{
		points[index].x = ControlPoints[j].x;
		points[index].y = ControlPoints[j].y;
		index++;
	}

	double coeffient;
	for (int j = 1; j <= k; j++) //k次循环
	{
		for (int n = i-k, p = 0; n <= i-j; n++)
		{
			coeffient = (t - NodeVector[n+j])/(NodeVector[n+k+1]-NodeVector[n+j]);
			
			points[p].x = ((1.0-coeffient)*points[p].x + coeffient*points[p+1].x); 
			points[p].y = ((1.0-coeffient)*points[p].y + coeffient*points[p+1].y);
			p++;
		}
	}

	CPoint2d returnPoint = points[0];
	delete []points;

	return returnPoint;
}


void Bspline::GetBSplinePnts()
{

	int conNum = ControlPoints.size();
 	
 	CPoint2d tempPoint;
	CPoint2d cpoint;
 	for (int i = 3; i <= conNum-1; i++)
 	{
 		double step = (NodeVector[i+1]-NodeVector[i])/DRAW_BSPLINE_STEP;
		if (step == 0)
		{
			continue;
		}
 		double initStep = NodeVector[i];
 		
 		for (int j = 1; j < DRAW_BSPLINE_STEP; j++)
 		{
 				tempPoint = GetdeBoorValue(i,initStep);
				//cpoint.x = (long)tempPoint.x;
				//cpoint.y = (long)tempPoint.y;
				cpoint.x = tempPoint.x;
				cpoint.y = tempPoint.y;
 				BSplinePoints.push_back(cpoint);
 				initStep += step;
 		}

		tempPoint = GetdeBoorValue(i, NodeVector[i+1]);

		//cpoint.x = (long)tempPoint.x;
		//cpoint.y = (long)tempPoint.y;

		cpoint.x = tempPoint.x;
		cpoint.y = tempPoint.y;

 		BSplinePoints.push_back(cpoint);
 	}

}
