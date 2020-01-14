#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include "iostream"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
using namespace std;
const double PI = 3.1415926;

const float scanPeriod = 0.1;  //
const int systemDelay = 20;  //初始时的延时，延时20帧
int systemInitCount = 0;
bool systemInited = false;

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudIn(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZINormal>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZINormal>());
pcl::PointCloud<pcl::PointXYZINormal>::Ptr cornerPointsSharp(new pcl::PointCloud<pcl::PointXYZINormal>());
pcl::PointCloud<pcl::PointXYZINormal>::Ptr cornerPointsLessSharp(new pcl::PointCloud<pcl::PointXYZINormal>());
pcl::PointCloud<pcl::PointXYZINormal>::Ptr surfPointsFlat(new pcl::PointCloud<pcl::PointXYZINormal>());
pcl::PointCloud<pcl::PointXYZINormal>::Ptr surfPointsLessFlat(new pcl::PointCloud<pcl::PointXYZINormal>());
pcl::PointCloud<pcl::PointXYZINormal>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<pcl::PointXYZINormal>());
pcl::PointCloud<pcl::PointXYZINormal>::Ptr surfPointsLessFlatScanDS(new pcl::PointCloud<pcl::PointXYZINormal>());
pcl::PointCloud<pcl::PointXYZ>::Ptr imuTrans(new pcl::PointCloud<pcl::PointXYZ>(4, 1));
pcl::PointCloud<pcl::PointXYZINormal>::Ptr laserCloudScans[16];

float cloudCurvature[40000];
int cloudSortInd[40000];
int cloudNeighborPicked[40000];
int cloudLabel[40000];

int scanStartInd[16];
int scanEndInd[16];

int imuPointerFront = 0;
int imuPointerLast = -1;
const int imuQueLength = 200;//imu队列大小。
bool imuInited=false;
float imuRollStart = 0, imuPitchStart = 0, imuYawStart = 0;
float imuRollCur = 0, imuPitchCur = 0, imuYawCur = 0;

float imuVeloXStart = 0, imuVeloYStart = 0, imuVeloZStart = 0;
float imuShiftXStart = 0, imuShiftYStart = 0, imuShiftZStart = 0;

float imuVeloXCur = 0, imuVeloYCur = 0, imuVeloZCur = 0;
float imuShiftXCur = 0, imuShiftYCur = 0, imuShiftZCur = 0;

float imuShiftFromStartXCur = 0, imuShiftFromStartYCur = 0, imuShiftFromStartZCur = 0;
float imuVeloFromStartXCur = 0, imuVeloFromStartYCur = 0, imuVeloFromStartZCur = 0;

//这块包括每秒内imu的时间戳，三角数据，加速度，速度，位移。
double imuTime[imuQueLength] = {0};
float imuRoll[imuQueLength] = {0};
float imuPitch[imuQueLength] = {0};
float imuYaw[imuQueLength] = {0};

float imuAccX[imuQueLength] = {0};
float imuAccY[imuQueLength] = {0};
float imuAccZ[imuQueLength] = {0};

float imuVeloX[imuQueLength] = {0};
float imuVeloY[imuQueLength] = {0};
float imuVeloZ[imuQueLength] = {0};

float imuShiftX[imuQueLength] = {0};
float imuShiftY[imuQueLength] = {0};
float imuShiftZ[imuQueLength] = {0};

ros::Publisher* pubLaserCloudPointer;
ros::Publisher* pubCornerPointsSharpPointer;
ros::Publisher* pubCornerPointsLessSharpPointer;
ros::Publisher* pubSurfPointsFlatPointer;
ros::Publisher* pubSurfPointsLessFlatPointer;
ros::Publisher* pubImuTransPointer;

//新特征点 
const int max_scan_seg_num = 20;
const int min_scan_seg_num = 6;
const int max_num = 2017*16;
int scan_seg_start[max_num];
int scan_seg_end[max_num];
int scan_seg_num = 0;


void ShiftToStartIMU(float pointTime)
{
	imuShiftFromStartXCur = imuShiftXCur - imuShiftXStart - imuVeloXStart * pointTime;
	imuShiftFromStartYCur = imuShiftYCur - imuShiftYStart - imuVeloYStart * pointTime;
	imuShiftFromStartZCur = imuShiftZCur - imuShiftZStart - imuVeloZStart * pointTime;

	float x1 = cos(imuYawStart) * imuShiftFromStartXCur - sin(imuYawStart) * imuShiftFromStartZCur;
	float y1 = imuShiftFromStartYCur;
	float z1 = sin(imuYawStart) * imuShiftFromStartXCur + cos(imuYawStart) * imuShiftFromStartZCur;

	float x2 = x1;
	float y2 = cos(imuPitchStart) * y1 + sin(imuPitchStart) * z1;
	float z2 = -sin(imuPitchStart) * y1 + cos(imuPitchStart) * z1;

	imuShiftFromStartXCur = cos(imuRollStart) * x2 + sin(imuRollStart) * y2;
	imuShiftFromStartYCur = -sin(imuRollStart) * x2 + cos(imuRollStart) * y2;
	imuShiftFromStartZCur = z2;
}

void VeloToStartIMU()
{
	imuVeloFromStartXCur = imuVeloXCur - imuVeloXStart;
	imuVeloFromStartYCur = imuVeloYCur - imuVeloYStart;
	imuVeloFromStartZCur = imuVeloZCur - imuVeloZStart;

	float x1 = cos(imuYawStart) * imuVeloFromStartXCur - sin(imuYawStart) * imuVeloFromStartZCur;
	float y1 = imuVeloFromStartYCur;
	float z1 = sin(imuYawStart) * imuVeloFromStartXCur + cos(imuYawStart) * imuVeloFromStartZCur;

	float x2 = x1;
	float y2 = cos(imuPitchStart) * y1 + sin(imuPitchStart) * z1;
	float z2 = -sin(imuPitchStart) * y1 + cos(imuPitchStart) * z1;

	imuVeloFromStartXCur = cos(imuRollStart) * x2 + sin(imuRollStart) * y2;
	imuVeloFromStartYCur = -sin(imuRollStart) * x2 + cos(imuRollStart) * y2;
	imuVeloFromStartZCur = z2;
}

void TransformToStartIMU(pcl::PointXYZINormal *p)
{
	float x1 = cos(imuRollCur) * p->x - sin(imuRollCur) * p->y;
	float y1 = sin(imuRollCur) * p->x + cos(imuRollCur) * p->y;
	float z1 = p->z;

	float x2 = x1;
	float y2 = cos(imuPitchCur) * y1 - sin(imuPitchCur) * z1;
	float z2 = sin(imuPitchCur) * y1 + cos(imuPitchCur) * z1;

	float x3 = cos(imuYawCur) * x2 + sin(imuYawCur) * z2;
	float y3 = y2;
	float z3 = -sin(imuYawCur) * x2 + cos(imuYawCur) * z2;

	float x4 = cos(imuYawStart) * x3 - sin(imuYawStart) * z3;
	float y4 = y3;
	float z4 = sin(imuYawStart) * x3 + cos(imuYawStart) * z3;

	float x5 = x4;
	float y5 = cos(imuPitchStart) * y4 + sin(imuPitchStart) * z4;
	float z5 = -sin(imuPitchStart) * y4 + cos(imuPitchStart) * z4;

	p->x = cos(imuRollStart) * x5 + sin(imuRollStart) * y5 + imuShiftFromStartXCur;
	p->y = -sin(imuRollStart) * x5 + cos(imuRollStart) * y5 + imuShiftFromStartYCur;
	p->z = z5 + imuShiftFromStartZCur;
}

void AccumulateIMUShift()
{
	//这个函数的作用：计算每一个imu数据处于的空间位置，速度，三个角
	float roll = imuRoll[imuPointerLast];
	float pitch = imuPitch[imuPointerLast];
	float yaw = imuYaw[imuPointerLast];
	float accX = imuAccX[imuPointerLast];
	float accY = imuAccY[imuPointerLast];
	float accZ = imuAccZ[imuPointerLast];

	float x1 = cos(roll) * accX - sin(roll) * accY;
	float y1 = sin(roll) * accX + cos(roll) * accY;
	float z1 = accZ;

	float x2 = x1;
	float y2 = cos(pitch) * y1 - sin(pitch) * z1;
	float z2 = sin(pitch) * y1 + cos(pitch) * z1;

	accX = cos(yaw) * x2 + sin(yaw) * z2;
	accY = y2;
	accZ = -sin(yaw) * x2 + cos(yaw) * z2;

	int imuPointerBack = (imuPointerLast + imuQueLength - 1) % imuQueLength;//前一帧
	double timeDiff = imuTime[imuPointerLast] - imuTime[imuPointerBack];//imu目前帧和前一帧的
	//这块的时间间隔
	if (timeDiff < 0.1)
	{
		imuShiftX[imuPointerLast] = imuShiftX[imuPointerBack] + imuVeloX[imuPointerBack] * timeDiff
							   + accX * timeDiff * timeDiff / 2;
		imuShiftY[imuPointerLast] = imuShiftY[imuPointerBack] + imuVeloY[imuPointerBack] * timeDiff
							   + accY * timeDiff * timeDiff / 2;
		imuShiftZ[imuPointerLast] = imuShiftZ[imuPointerBack] + imuVeloZ[imuPointerBack] * timeDiff
							   + accZ * timeDiff * timeDiff / 2;

		imuVeloX[imuPointerLast] = imuVeloX[imuPointerBack] + accX * timeDiff;
		imuVeloY[imuPointerLast] = imuVeloY[imuPointerBack] + accY * timeDiff;
		imuVeloZ[imuPointerLast] = imuVeloZ[imuPointerBack] + accZ * timeDiff;
	}
}

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudIn2)
{
	//这里有一个输入点云延时的处理，当进来的点云超过20个时，再进行进行接下来的处理
	if (!systemInited)
	{
		systemInitCount++;
		if (systemInitCount >= systemDelay)
		{
			systemInited = true;
		}
		return;
	}

	double timeScanCur = laserCloudIn2->header.stamp.toSec();

	//去除无效点
	pcl::fromROSMsg(*laserCloudIn2, *laserCloudIn);
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*laserCloudIn,*laserCloudIn, indices);
	indices.clear();
	int cloudSize = laserCloudIn->points.size();
	
	//进来点云的起始角度和终止角度
	float startOri = -atan2(laserCloudIn->points[0].y, laserCloudIn->points[0].x);
	float endOri = -atan2(laserCloudIn->points[cloudSize - 1].y,
						 laserCloudIn->points[cloudSize - 1].x) + 2 * PI;
	//把角度限定在pi-2pi之内
	if (endOri - startOri > 3 * PI)
	{
		endOri -= 2 * PI;
	}
	else if (endOri - startOri < PI)
	{
		endOri += 2 * PI;
	}
   
	//将点云数据复制给point,注意变换为视觉坐标系
	/*
	x-y
	y-z
	z-x
	*/
	bool halfPassed = false;
	int count = cloudSize;
	pcl::PointXYZINormal point;
	for (int i = 0; i < cloudSize; i++)
	{
		point.x = laserCloudIn->points[i].y;
		point.y = laserCloudIn->points[i].z;
		point.z = laserCloudIn->points[i].x;
		point.curvature = laserCloudIn->points[i].intensity;
		
		//滤除有效距离外的点
		float dis2orig2 = point.x*point.x + point.y*point.y +point.z*point.z;
		if(dis2orig2<0.04 || dis2orig2>4900.0)
		{
			count--;
			continue;
		}
	
		//计算当前点属于哪条线，范围 0-15
		float angle = atan(point.y / sqrt(point.x * point.x + point.z * point.z)) * 180 / PI;
		int scanID;
		int angle2 = int(angle + (angle<0.0?-0.5:+0.5));
		if(angle2>0)
		{
			scanID = angle2/2 + 8;
		}
		else
		{
			scanID = angle2/2 + 7;
		}
		if (scanID > 15 )
		{
			count--;
			continue;
		}
		
		//当前点位于一条扫描线的什么角度
		float ori = -atan2(point.x, point.z);
		if (!halfPassed)
		{
			if (ori < startOri - PI / 2) {
				ori += 2 * PI;
			}
			else if (ori > startOri + PI * 3 / 2) {
				ori -= 2 * PI;
			}

			if (ori - startOri > PI){
				halfPassed = true;
			}
		}
		else
		{
			ori += 2 * PI;
			if (ori < endOri - PI * 3 / 2) {
				ori += 2 * PI;
			}
			else if (ori > endOri + PI / 2) {
				ori -= 2 * PI;
			}
		}
		
		//intensity被改写了，这个值能确定该点在什么位置，第几条线的什么角度
		float relTime = (ori - startOri) / (endOri - startOri);
		point.intensity = scanID + 0.1 * relTime;


		//关于IMU的：动态补偿+
		if (imuPointerLast >= 0)
		{
			float pointTime = relTime * scanPeriod;//算了该点所处一个周期内什么时刻
			while (imuPointerFront != imuPointerLast)
			{
				if (timeScanCur + pointTime < imuTime[imuPointerFront]) {
					break;
				}
				imuPointerFront = (imuPointerFront + 1) % imuQueLength;
			}

			if (timeScanCur + pointTime > imuTime[imuPointerFront])
			{
				imuRollCur = imuRoll[imuPointerFront];
				imuPitchCur = imuPitch[imuPointerFront];
				imuYawCur = imuYaw[imuPointerFront];

				imuVeloXCur = imuVeloX[imuPointerFront];
				imuVeloYCur = imuVeloY[imuPointerFront];
				imuVeloZCur = imuVeloZ[imuPointerFront];

				imuShiftXCur = imuShiftX[imuPointerFront];
				imuShiftYCur = imuShiftY[imuPointerFront];
				imuShiftZCur = imuShiftZ[imuPointerFront];
			}
			else
			{
				int imuPointerBack = (imuPointerFront + imuQueLength - 1) % imuQueLength;
				float ratioFront = (timeScanCur + pointTime - imuTime[imuPointerBack])
								/ (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
				float ratioBack = (imuTime[imuPointerFront] - timeScanCur - pointTime)
								/ (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
								
				imuRollCur = imuRoll[imuPointerFront] * ratioFront + imuRoll[imuPointerBack] * ratioBack;
				imuPitchCur = imuPitch[imuPointerFront] * ratioFront + imuPitch[imuPointerBack] * ratioBack;
				if (imuYaw[imuPointerFront] - imuYaw[imuPointerBack] > PI)
				{
					imuYawCur = imuYaw[imuPointerFront] * ratioFront + (imuYaw[imuPointerBack] + 2 * PI) * ratioBack;
				}
				else if (imuYaw[imuPointerFront] - imuYaw[imuPointerBack] < -PI)
				{
					imuYawCur = imuYaw[imuPointerFront] * ratioFront + (imuYaw[imuPointerBack] - 2 * PI) * ratioBack;
				}
				else
				{
					imuYawCur = imuYaw[imuPointerFront] * ratioFront + imuYaw[imuPointerBack] * ratioBack;
				}

				imuVeloXCur = imuVeloX[imuPointerFront] * ratioFront + imuVeloX[imuPointerBack] * ratioBack;
				imuVeloYCur = imuVeloY[imuPointerFront] * ratioFront + imuVeloY[imuPointerBack] * ratioBack;
				imuVeloZCur = imuVeloZ[imuPointerFront] * ratioFront + imuVeloZ[imuPointerBack] * ratioBack;

				imuShiftXCur = imuShiftX[imuPointerFront] * ratioFront + imuShiftX[imuPointerBack] * ratioBack;
				imuShiftYCur = imuShiftY[imuPointerFront] * ratioFront + imuShiftY[imuPointerBack] * ratioBack;
				imuShiftZCur = imuShiftZ[imuPointerFront] * ratioFront + imuShiftZ[imuPointerBack] * ratioBack;
			}

			if (i==0)
			{
				imuRollStart = imuRollCur;
				imuPitchStart = imuPitchCur;
				imuYawStart = imuYawCur;

				imuVeloXStart = imuVeloXCur;
				imuVeloYStart = imuVeloYCur;
				imuVeloZStart = imuVeloZCur;

				imuShiftXStart = imuShiftXCur;
				imuShiftYStart = imuShiftYCur;
				imuShiftZStart = imuShiftZCur;

				imuInited=true;
			}
			else
			{
				ShiftToStartIMU(pointTime);
				VeloToStartIMU();
				TransformToStartIMU(&point);
			}
		}

		laserCloudScans[scanID]->push_back(point);
	}

	

	//整理点云，准备提取特征点
    for (int i = 0; i < 16; i++)
    {
		if(laserCloudScans[i]->points.size() <100)
		{
			scanStartInd[i] = 0;
			scanEndInd[i] = 0;
			continue;
		}
		
		scanStartInd[i] = laserCloud->points.size() + 5;  //i扫描线在一维数组中的起点
		*laserCloud += *laserCloudScans[i];  //放到一个数组中
		scanEndInd[i] = laserCloud->points.size() - 5;  //i扫描线在一维数组中的终点
    }
	cloudSize = laserCloud->size();
	
	
	//开始计算特征点
	//计算c值 s表示i点周围10个点，前5后5
	int scanCount = -1;
	for (int i = 5; i < cloudSize - 5; i++)
	{
		float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x
				 + laserCloud->points[i - 3].x + laserCloud->points[i - 2].x
				 + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x
				 + laserCloud->points[i + 1].x + laserCloud->points[i + 2].x
				 + laserCloud->points[i + 3].x + laserCloud->points[i + 4].x
				 + laserCloud->points[i + 5].x;
		float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y
				 + laserCloud->points[i - 3].y + laserCloud->points[i - 2].y
				 + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y
				 + laserCloud->points[i + 1].y + laserCloud->points[i + 2].y
				 + laserCloud->points[i + 3].y + laserCloud->points[i + 4].y
				 + laserCloud->points[i + 5].y;
		float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z
				 + laserCloud->points[i - 3].z + laserCloud->points[i - 2].z
				 + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z
				 + laserCloud->points[i + 1].z + laserCloud->points[i + 2].z
				 + laserCloud->points[i + 3].z + laserCloud->points[i + 4].z
				 + laserCloud->points[i + 5].z;

		cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;//点云曲率
		cloudSortInd[i] = i;//排序
		cloudNeighborPicked[i] = 0;//
		cloudLabel[i] = 0;
	}

	//消除遮挡等
	for (int i = 5; i < cloudSize - 6; i++)
	{
		float diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x;
		float diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y;
		float diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z;
		float diff = diffX * diffX + diffY * diffY + diffZ * diffZ;

		if (diff > 0.1)
		{
			//用于消去拐点.特征为遮挡的情况
			//点的深度

			float depth12 = laserCloud->points[i].x * laserCloud->points[i].x +
								laserCloud->points[i].y * laserCloud->points[i].y +
								laserCloud->points[i].z * laserCloud->points[i].z;

			float depth22 = laserCloud->points[i + 1].x * laserCloud->points[i + 1].x +
								laserCloud->points[i + 1].y * laserCloud->points[i + 1].y +
								laserCloud->points[i + 1].z * laserCloud->points[i + 1].z;

			if (depth12 > depth22)
			{
				float bilv = sqrt(depth22 / depth12);
				diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x * bilv;
				diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y * bilv;
				diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z * bilv;

				if ((diffX * diffX + diffY * diffY + diffZ * diffZ)  < 0.01*depth22)
				{
					cloudNeighborPicked[i - 5] = 1;
					cloudNeighborPicked[i - 4] = 1;
					cloudNeighborPicked[i - 3] = 1;
					cloudNeighborPicked[i - 2] = 1;
					cloudNeighborPicked[i - 1] = 1;
					cloudNeighborPicked[i] = 1;
				}
			}
			else
			{
				float bilv = sqrt(depth12 / depth22);
				diffX = laserCloud->points[i + 1].x * bilv - laserCloud->points[i].x;
				diffY = laserCloud->points[i + 1].y * bilv - laserCloud->points[i].y;
				diffZ = laserCloud->points[i + 1].z * bilv - laserCloud->points[i].z;

				if ((diffX * diffX + diffY * diffY + diffZ * diffZ)   < 0.01*depth12)
				{
					cloudNeighborPicked[i + 1] = 1;
					cloudNeighborPicked[i + 2] = 1;
					cloudNeighborPicked[i + 3] = 1;
					cloudNeighborPicked[i + 4] = 1;
					cloudNeighborPicked[i + 5] = 1;
					cloudNeighborPicked[i + 6] = 1;
				}
			}
		}

		float diffX2 = laserCloud->points[i].x - laserCloud->points[i - 1].x;
		float diffY2 = laserCloud->points[i].y - laserCloud->points[i - 1].y;
		float diffZ2 = laserCloud->points[i].z - laserCloud->points[i - 1].z;
		float diff2 = diffX2 * diffX2 + diffY2 * diffY2 + diffZ2 * diffZ2;

		float dis = laserCloud->points[i].x * laserCloud->points[i].x
			   + laserCloud->points[i].y * laserCloud->points[i].y
			   + laserCloud->points[i].z * laserCloud->points[i].z;
		
		//消除斜率太大的情况
		if (diff > 0.0002 * dis && diff2 > 0.0002 * dis)
		{
			cloudNeighborPicked[i] = 1;
		}
	}


	
	
	//=============== 原来的特征提取 ==========
	for (int i = 0; i < 16; i++)
	{
		surfPointsLessFlatScan->clear();
		//将一条线分为6段，每一段都找到了两个最大的角点和四个最小的平面
		for (int j = 0; j < 6; j++)
		{
			int sp = (scanStartInd[i] * (6 - j)  + scanEndInd[i] * j) / 6;
			int ep = (scanStartInd[i] * (5 - j)  + scanEndInd[i] * (j + 1)) / 6 - 1;
			//升序排序
			for (int k = sp + 1; k <= ep; k++)
			{
				for (int l = k; l >= sp + 1; l--)
				{
					if (cloudCurvature[cloudSortInd[l]] < cloudCurvature[cloudSortInd[l - 1]])
					{
						int temp = cloudSortInd[l - 1];
						cloudSortInd[l - 1] = cloudSortInd[l];
						cloudSortInd[l] = temp;
					}
				}
			}
       
			//选取角点
			int largestPickedNum = 0;
			for (int k = ep; k >= sp; k--)
			{
				int ind = cloudSortInd[k];
				if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > 0.1)
				{
					largestPickedNum++;
					if (largestPickedNum <= 2)
					{
						cloudLabel[ind] = 2; //点云标志，定义角点、面点。2-最角点，1-次角点
						cornerPointsSharp->push_back(laserCloud->points[ind]);
						cornerPointsLessSharp->push_back(laserCloud->points[ind]);
					}
					else if (largestPickedNum <= 20)
					{
						cloudLabel[ind] = 1;//次角点
						cornerPointsLessSharp->push_back(laserCloud->points[ind]);
					}
					else
					{
						break;
					}

					cloudNeighborPicked[ind] = 1;
					for (int l = 1; l <= 5; l++)
					{
						float diffX = laserCloud->points[ind + l].x
									- laserCloud->points[ind + l - 1].x;
						float diffY = laserCloud->points[ind + l].y
									- laserCloud->points[ind + l - 1].y;
						float diffZ = laserCloud->points[ind + l].z
									- laserCloud->points[ind + l - 1].z;
						if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
						{
							break;
						}
						cloudNeighborPicked[ind + l] = 1;
					}
					for (int l = -1; l >= -5; l--)
					{
						float diffX = laserCloud->points[ind + l].x
									- laserCloud->points[ind + l + 1].x;
						float diffY = laserCloud->points[ind + l].y
									- laserCloud->points[ind + l + 1].y;
						float diffZ = laserCloud->points[ind + l].z
									- laserCloud->points[ind + l + 1].z;
						if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
						{
							break;
						}

						cloudNeighborPicked[ind + l] = 1;
					}
				}
			}
			
			
			//选取平面点
			int smallestPickedNum = 0;
			for (int k = sp; k <= ep; k++)
			{
				int ind = cloudSortInd[k];
				if (cloudNeighborPicked[ind] == 0 &&cloudCurvature[ind] < 0.1)
				{
					cloudLabel[ind] = -1;//面性点的标志 -1
					surfPointsFlat->push_back(laserCloud->points[ind]);
					smallestPickedNum++;
					if (smallestPickedNum >= 4)
					{
						//选取4个
						break;
					}

					cloudNeighborPicked[ind] = 1;
					for (int l = 1; l <= 5; l++)
					{
						float diffX = laserCloud->points[ind + l].x
									- laserCloud->points[ind + l - 1].x;
						float diffY = laserCloud->points[ind + l].y
									- laserCloud->points[ind + l - 1].y;
						float diffZ = laserCloud->points[ind + l].z
									- laserCloud->points[ind + l - 1].z;
						if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
						{
							break;
						}

						cloudNeighborPicked[ind + l] = 1;
					}
					for (int l = -1; l >= -5; l--)
					{
						if((ind+l) < 0)
							continue;
						
						float diffX = laserCloud->points[ind + l].x
									- laserCloud->points[ind + l + 1].x;
						float diffY = laserCloud->points[ind + l].y
									- laserCloud->points[ind + l + 1].y;
						float diffZ = laserCloud->points[ind + l].z
									- laserCloud->points[ind + l + 1].z;
						if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
						{
							break;
						}

						cloudNeighborPicked[ind + l] = 1;
					}
				}
			}

			for (int k = sp; k <= ep; k++)
			{
				if (cloudLabel[k] <= 0)
				{
					surfPointsLessFlatScan->push_back(laserCloud->points[k]);
				}
			}
		}

		surfPointsLessFlatScanDS->clear();
		pcl::VoxelGrid<pcl::PointXYZINormal> downSizeFilter;
		downSizeFilter.setInputCloud(surfPointsLessFlatScan);
		downSizeFilter.setLeafSize(0.2, 0.2, 0.2);  //平面点降采样
		downSizeFilter.filter(*surfPointsLessFlatScanDS);

		*surfPointsLessFlat += *surfPointsLessFlatScanDS;
	}
	

	sensor_msgs::PointCloud2 laserCloud2;
	pcl::toROSMsg(*laserCloud, laserCloud2);
	laserCloud2.header.stamp = laserCloudIn2->header.stamp;
	laserCloud2.header.frame_id = "/camera";
	pubLaserCloudPointer->publish(laserCloud2);

	sensor_msgs::PointCloud2 cornerPointsSharp2;
	pcl::toROSMsg(*cornerPointsSharp, cornerPointsSharp2);
	cornerPointsSharp2.header.stamp = laserCloudIn2->header.stamp;
	cornerPointsSharp2.header.frame_id = "/camera";
	pubCornerPointsSharpPointer->publish(cornerPointsSharp2);

	sensor_msgs::PointCloud2 cornerPointsLessSharp2;
	pcl::toROSMsg(*cornerPointsLessSharp, cornerPointsLessSharp2);
	cornerPointsLessSharp2.header.stamp = laserCloudIn2->header.stamp;
	cornerPointsLessSharp2.header.frame_id = "/camera";
	pubCornerPointsLessSharpPointer->publish(cornerPointsLessSharp2);

	sensor_msgs::PointCloud2 surfPointsFlat2;
	pcl::toROSMsg(*surfPointsFlat, surfPointsFlat2);
	surfPointsFlat2.header.stamp = laserCloudIn2->header.stamp;
	surfPointsFlat2.header.frame_id = "/camera";
	pubSurfPointsFlatPointer->publish(surfPointsFlat2);

	sensor_msgs::PointCloud2 surfPointsLessFlat2;
	pcl::toROSMsg(*surfPointsLessFlat, surfPointsLessFlat2);
	surfPointsLessFlat2.header.stamp = laserCloudIn2->header.stamp;
	surfPointsLessFlat2.header.frame_id = "/camera";
	pubSurfPointsLessFlatPointer->publish(surfPointsLessFlat2);
	//std::cout << "输出imu_trans数据" << std::endl;
	imuTrans->points[0].x = imuPitchStart;
	imuTrans->points[0].y = imuYawStart;
	imuTrans->points[0].z = imuRollStart;
	//std::cout <<imuPitchStart<<","<<imuYawStart<<","<<imuRollStart<<std::endl;
	imuTrans->points[1].x = imuPitchCur;
	imuTrans->points[1].y = imuYawCur;
	imuTrans->points[1].z = imuRollCur;
	//std::cout <<imuPitchCur<<","<<imuYawCur<<","<<imuRollCur<<std::endl;
	imuTrans->points[2].x = imuShiftFromStartXCur;
	imuTrans->points[2].y = imuShiftFromStartYCur;
	imuTrans->points[2].z = imuShiftFromStartZCur;
	//std::cout <<imuShiftFromStartXCur<<","<<imuShiftFromStartYCur<<","<<imuShiftFromStartZCur<<std::endl;
	imuTrans->points[3].x = imuVeloFromStartXCur;
	imuTrans->points[3].y = imuVeloFromStartYCur;
	imuTrans->points[3].z = imuVeloFromStartZCur;
	//std::cout <<imuVeloFromStartXCur<<","<<imuVeloFromStartYCur<<","<<imuVeloFromStartZCur<<std::endl;
	//std::cout <<std::endl;
	sensor_msgs::PointCloud2 imuTrans2;
	pcl::toROSMsg(*imuTrans, imuTrans2);
	imuTrans2.header.stamp = laserCloudIn2->header.stamp;
	imuTrans2.header.frame_id = "/camera";
	pubImuTransPointer->publish(imuTrans2);

	laserCloudIn->clear();
	laserCloud->clear();
	cornerPointsSharp->clear();
	cornerPointsLessSharp->clear();
	surfPointsFlat->clear();
	surfPointsLessFlat->clear();

	for (int i = 0; i < 16; i++)
	{
		laserCloudScans[i]->points.clear();
	}
}

void imuHandler(const sensor_msgs::Imu::ConstPtr& imuIn)
{
	double roll, pitch, yaw;
	tf::Quaternion orientation;
	tf::quaternionMsgToTF(imuIn->orientation, orientation);
	tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
	// std::cout << roll << "," << pitch << "," << yaw << std::endl;
	/*
	x-y
	y-z
	z-x
	*/
	float accX = imuIn->linear_acceleration.y - sin(roll) * cos(pitch) * 9.81;
	float accY = imuIn->linear_acceleration.z - cos(roll) * cos(pitch) * 9.81;
	float accZ = imuIn->linear_acceleration.x + sin(pitch) * 9.81;
	//std::cout << accX <<","<< accY <<","<< accZ << std::endl;

	imuPointerLast = (imuPointerLast + 1) % imuQueLength;

	imuTime[imuPointerLast] = imuIn->header.stamp.toSec();
	imuRoll[imuPointerLast] = roll;
	imuPitch[imuPointerLast] = pitch;
	imuYaw[imuPointerLast] = yaw;
	imuAccX[imuPointerLast] = accX;
	imuAccY[imuPointerLast] = accY;
	imuAccZ[imuPointerLast] = accZ;

	AccumulateIMUShift();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "register");
	ros::NodeHandle nh;

	for (int i = 0; i < 16; i++) {
		laserCloudScans[i].reset(new pcl::PointCloud<pcl::PointXYZINormal>());
	}

	//预先申请内存，提升效率
	laserCloudIn->reserve(289000);
	laserCloud->reserve(289000);
	cornerPointsSharp->reserve(200);
	cornerPointsLessSharp->reserve(289000);
	surfPointsFlat->reserve(400);
	surfPointsLessFlat->reserve(289000);
	for (int i = 0; i < 16; i++)
	{
		laserCloudScans[i]->reserve(2000);
	}
	surfPointsLessFlatScanDS->reserve(2000); //每条线上的平面点
	surfPointsLessFlatScan->reserve(2000);


	ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>
								   ("/velodyne_points", 2, laserCloudHandler);
	//ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>
	//							   ("/rslidar_points", 2, laserCloudHandler);

	ros::Subscriber subImu = nh.subscribe<sensor_msgs::Imu> ("/imu/data", 50, imuHandler);

	ros::Publisher pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>
								  ("/velodyne_cloud_2", 2);

	ros::Publisher pubCornerPointsSharp = nh.advertise<sensor_msgs::PointCloud2>
										 ("/laser_cloud_sharp", 2);

	ros::Publisher pubCornerPointsLessSharp = nh.advertise<sensor_msgs::PointCloud2>
											 ("/laser_cloud_less_sharp", 2);

	ros::Publisher pubSurfPointsFlat = nh.advertise<sensor_msgs::PointCloud2>
										("/laser_cloud_flat", 2);

	ros::Publisher pubSurfPointsLessFlat = nh.advertise<sensor_msgs::PointCloud2>
											("/laser_cloud_less_flat", 2);

	ros::Publisher pubImuTrans = nh.advertise<sensor_msgs::PointCloud2> ("/imu_trans", 5);

	pubLaserCloudPointer = &pubLaserCloud;
	pubCornerPointsSharpPointer = &pubCornerPointsSharp;
	pubCornerPointsLessSharpPointer = &pubCornerPointsLessSharp;
	pubSurfPointsFlatPointer = &pubSurfPointsFlat;
	pubSurfPointsLessFlatPointer = &pubSurfPointsLessFlat;
	pubImuTransPointer = &pubImuTrans;

	ros::spin();

	return 0;
}
