#include "path_planner/point.h"
#include "path_planner/smooth.h"
CBSpline::CBSpline(int power, std::vector<CPoint2d>& PtArray)
: Power(power)
, StepofDeBoor(1)
{
	PointCount = PtArray.size();
	PointArray = PtArray;
	SpotVector = GetSpotVector(PointCount, Power);
}

CBSpline::~CBSpline(void)
{
	SpotVector.clear();
	PointArray.clear();
}

//de boor
std::vector<CPoint2d>& CBSpline::GetBSplinePoint()
{
	for (int i = Power; i < PointCount; i++)
	{
		double u = SpotVector[i];
		while (u <= SpotVector[i + 1])
		{
			StepofDeBoor = 1;
			std::vector<CPoint2d> temp;
			for (int n = i - Power; n <= i; n++)
				temp.push_back(PointArray[n]);
			dComputing(temp, i, u);
			if (u < SpotVector[i + 1] - 0.98)
				u += 0.005;
			else
				u += 0.005;
		}
	}
	return PointLine;
}

double CBSpline::Alpha(int index, double U, int step)
{
	double temp = (U - SpotVector.at(index)) / (SpotVector.at(index + Power + 1 -step) - SpotVector.at(index));
	if (SpotVector.at(index + Power + 1 - step) - SpotVector.at(index) == 0)
		temp = 0.01;
	return temp;
}

void CBSpline::dComputing(std::vector<CPoint2d> &PtAry, int i, double u)
{
	if (StepofDeBoor <= Power)
	{
		int j = i - Power + StepofDeBoor;
		CPoint2d MidPoint;
		std::vector<CPoint2d> PtAryNew;
		double alph;	// = Alpha(j, u, StepofDeBoor);
		for (int a = 1; a < PtAry.size(); a++)
		{
			alph = Alpha(j, u, StepofDeBoor);
			MidPoint.x = (1.0 - alph) * PtAry[a - 1].x + alph * PtAry[a].x;
			MidPoint.y = (1.0 - alph) * PtAry[a - 1].y + alph * PtAry[a].y;
			PtAryNew.push_back(MidPoint);
			j++;
			if (StepofDeBoor == Power)
				PointLine.push_back(MidPoint);
		}
		PtAry = PtAryNew;
		StepofDeBoor++;
		dComputing(PtAry, i, u);
	}
}


std::vector<double> CBSpline::GetSpotVector(int PointCount, int Power)
{
	std::vector<double> Uvec;

	//拟标准样条
	for (int i = 0; i < PointCount + Power + 1; i++)
	{
		if (i < Power)
			Uvec.push_back(0);
		if (Power <= i && i <= PointCount)
			Uvec.push_back(((double)(- Power + i)) / (PointCount - Power));
		if (i > PointCount)
			Uvec.push_back(1);
	}
	return Uvec;

}

CBezier::CBezier()
{

}

CBezier::~CBezier()
{

}

void CBezier::Bezier( std::vector<CPoint2d> &OriPoints,std::vector<CPoint2d> &NewPoints )
{
	CPoint2d pt0,pt1,pt2,pt3;
	pt0=OriPoints[0];
	pt1=OriPoints[1];
	pt2=OriPoints[2];
	pt3=OriPoints[3];

	int A0,A1,A2,A3,B0,B1,B2,B3;
	A0=pt0.x;
	A1=3*pt1.x-3*pt0.x;
	A2=3*pt2.x-6*pt1.x+3*pt0.x;
	A3=pt3.x-3*pt2.x+3*pt1.x-pt0.x;
	B0=pt0.y;
	B1=3*pt1.y-3*pt0.y;
	B2=3*pt2.y-6*pt1.y+3*pt0.y;
	B3=pt3.y-3*pt2.y+3*pt1.y-pt0.y;

	for (double t=0;t< 1;t+=0.001)
	{
		CPoint2d drawpt;
		drawpt.x=A0+(A1+(A2+A3*t)*t)*t;
		drawpt.y=B0+(B1+(B2+B3*t)*t)*t;
		NewPoints.push_back(drawpt);
	}
}
