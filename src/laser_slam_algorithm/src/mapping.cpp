#include <ros/ros.h>
#include <math.h>
#include <time.h>
#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <string>
using namespace std;

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

#include "laser_slam_algorithm/srv_record_map.h"

const double PI = 3.1415926;
const float scanPeriod = 0.1;

const int stackFrameNum = 1;
const int mapFrameNum = 5;

double timeLaserCloudCornerLast = 0;
double timeLaserCloudSurfLast = 0;
double timeLaserCloudFullRes = 0;
double timeLaserOdometry = 0;

bool newLaserCloudCornerLast = false;
bool newLaserCloudSurfLast = false;
bool newLaserCloudFullRes = false;
bool newLaserOdometry = false;

int laserCloudCenWidth = 6;  //这几个值是一直变的，起到一个偏移左右，它是真正相对于全局的
int laserCloudCenHeight = 2;
int laserCloudCenDepth = 6;
const int laserCloudWidth = 13;    //栅格个数
const int laserCloudHeight = 5;
const int laserCloudDepth = 13;
const int laserCloudNum = laserCloudWidth * laserCloudHeight * laserCloudDepth;
const float voxel_range = 30.0;  //栅格大小

int laserCloudValidInd[125];
int laserCloudSurroundInd[125];

pcl::PointCloud<pcl::PointXYZINormal>::Ptr priorSurfFeature(new pcl::PointCloud<pcl::PointXYZINormal>());//先验地图平面点
pcl::PointCloud<pcl::PointXYZINormal>::Ptr priorCornerFeature(new pcl::PointCloud<pcl::PointXYZINormal>());//先验地图边缘点

pcl::PointCloud<pcl::PointXYZINormal>::Ptr laserCloudCornerLast(new pcl::PointCloud<pcl::PointXYZINormal>());
pcl::PointCloud<pcl::PointXYZINormal>::Ptr laserCloudSurfLast(new pcl::PointCloud<pcl::PointXYZINormal>());
pcl::PointCloud<pcl::PointXYZINormal>::Ptr laserCloudCornerStack(new pcl::PointCloud<pcl::PointXYZINormal>());
pcl::PointCloud<pcl::PointXYZINormal>::Ptr laserCloudSurfStack(new pcl::PointCloud<pcl::PointXYZINormal>());
pcl::PointCloud<pcl::PointXYZINormal>::Ptr laserCloudCornerStack2(new pcl::PointCloud<pcl::PointXYZINormal>());
pcl::PointCloud<pcl::PointXYZINormal>::Ptr laserCloudSurfStack2(new pcl::PointCloud<pcl::PointXYZINormal>());
pcl::PointCloud<pcl::PointXYZINormal>::Ptr laserCloudOri(new pcl::PointCloud<pcl::PointXYZINormal>());
//pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSel(new pcl::PointCloud<pcl::PointXYZI>());
//pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCorr(new pcl::PointCloud<pcl::PointXYZI>());
//pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudProj(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZINormal>::Ptr coeffSel(new pcl::PointCloud<pcl::PointXYZINormal>());
pcl::PointCloud<pcl::PointXYZINormal>::Ptr laserCloudSurround(new pcl::PointCloud<pcl::PointXYZINormal>());
pcl::PointCloud<pcl::PointXYZINormal>::Ptr laserCloudSurround2(new pcl::PointCloud<pcl::PointXYZINormal>());
pcl::PointCloud<pcl::PointXYZINormal>::Ptr laserCloudCornerFromMap(new pcl::PointCloud<pcl::PointXYZINormal>());
pcl::PointCloud<pcl::PointXYZINormal>::Ptr laserCloudSurfFromMap(new pcl::PointCloud<pcl::PointXYZINormal>());
pcl::PointCloud<pcl::PointXYZINormal>::Ptr laserCloudFullRes(new pcl::PointCloud<pcl::PointXYZINormal>());
pcl::PointCloud<pcl::PointXYZINormal>::Ptr laserCloudCornerArray[laserCloudNum];
pcl::PointCloud<pcl::PointXYZINormal>::Ptr laserCloudSurfArray[laserCloudNum];
pcl::PointCloud<pcl::PointXYZINormal>::Ptr laserCloudCornerArray2[laserCloudNum];
pcl::PointCloud<pcl::PointXYZINormal>::Ptr laserCloudSurfArray2[laserCloudNum];

pcl::KdTreeFLANN<pcl::PointXYZINormal>::Ptr kdtreeCornerFromMap(new pcl::KdTreeFLANN<pcl::PointXYZINormal>());
pcl::KdTreeFLANN<pcl::PointXYZINormal>::Ptr kdtreeSurfFromMap(new pcl::KdTreeFLANN<pcl::PointXYZINormal>());

float transformSum[6] = {0};
float transformIncre[6] = {0};
float transformTobeMapped[6] = {0};
float transformBefMapped[6] = {0};
float transformAftMapped[6] = {0};

int imuPointerFront = 0;
int imuPointerLast = -1;
const int imuQueLength = 200;

double imuTime[imuQueLength] = {0};
float imuRoll[imuQueLength] = {0};
float imuPitch[imuQueLength] = {0};



bool is_record_map = false;
string surf_bfile;//平面点文件名
string corner_bfile;//边缘点文件名

struct VMapPoint{
    float x;
    float y;
    float z;
};

//保存两类特征点
void save_surf_point_for_map(pcl::PointXYZINormal data)
{
    ofstream fout(surf_bfile.c_str(), ios::app | ios ::binary | ios::out);
    if(!fout.is_open())
        return;
        
    VMapPoint tmp;
    tmp.x = data.x;
    tmp.y = data.y;
    tmp.z = data.z; 

    fout.write((char*)&tmp ,sizeof(VMapPoint));
    fout.close();
}

void save_corner_point_for_map(pcl::PointXYZINormal data)
{
    ofstream fout(corner_bfile.c_str(), ios::app | ios ::binary | ios::out);
    if(!fout.is_open())
        return;
        
    VMapPoint tmp;
    tmp.x = data.x;
    tmp.y = data.y;
    tmp.z = data.z; 

    fout.write((char*)&tmp ,sizeof(VMapPoint));
    fout.close();
}

//服务回调函数，是否存储特征点map
bool recordMapHandler(laser_slam_algorithm::srv_record_map::Request &req,
				     laser_slam_algorithm::srv_record_map::Response &res)
{
    surf_bfile.clear();
    corner_bfile.clear();
    
	if(req.is_record_map)
	{
		is_record_map = true;
		surf_bfile = "/home/robot/project_ws/map/" + req.str_map_name + "_surf.map";
		corner_bfile = "/home/robot/project_ws/map/" + req.str_map_name + "_corner.map";
		res.feedback = "Start to save  priormap!";  
	}
	else
	{
		is_record_map = false;
		res.feedback = "Stop to save  priormap!";  
	}
	
    return true;
}


void transformAssociateToMap()
{
   float x1 = cos(transformSum[1]) * (transformBefMapped[3] - transformSum[3])
            - sin(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);
   float y1 = transformBefMapped[4] - transformSum[4];
   float z1 = sin(transformSum[1]) * (transformBefMapped[3] - transformSum[3])
            + cos(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);

   float x2 = x1;
   float y2 = cos(transformSum[0]) * y1 + sin(transformSum[0]) * z1;
   float z2 = -sin(transformSum[0]) * y1 + cos(transformSum[0]) * z1;

   transformIncre[3] = cos(transformSum[2]) * x2 + sin(transformSum[2]) * y2;
   transformIncre[4] = -sin(transformSum[2]) * x2 + cos(transformSum[2]) * y2;
   transformIncre[5] = z2;

   float sbcx = sin(transformSum[0]);
   float cbcx = cos(transformSum[0]);
   float sbcy = sin(transformSum[1]);
   float cbcy = cos(transformSum[1]);
   float sbcz = sin(transformSum[2]);
   float cbcz = cos(transformSum[2]);

   float sblx = sin(transformBefMapped[0]);
   float cblx = cos(transformBefMapped[0]);
   float sbly = sin(transformBefMapped[1]);
   float cbly = cos(transformBefMapped[1]);
   float sblz = sin(transformBefMapped[2]);
   float cblz = cos(transformBefMapped[2]);

   float salx = sin(transformAftMapped[0]);
   float calx = cos(transformAftMapped[0]);
   float saly = sin(transformAftMapped[1]);
   float caly = cos(transformAftMapped[1]);
   float salz = sin(transformAftMapped[2]);
   float calz = cos(transformAftMapped[2]);

   float srx = -sbcx*(salx*sblx + calx*caly*cblx*cbly + calx*cblx*saly*sbly)
             - cbcx*cbcz*(calx*saly*(cbly*sblz - cblz*sblx*sbly)
             - calx*caly*(sbly*sblz + cbly*cblz*sblx) + cblx*cblz*salx)
             - cbcx*sbcz*(calx*caly*(cblz*sbly - cbly*sblx*sblz)
             - calx*saly*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sblz);
   transformTobeMapped[0] = -asin(srx);

   float srycrx = (cbcy*sbcz - cbcz*sbcx*sbcy)*(calx*saly*(cbly*sblz - cblz*sblx*sbly)
                - calx*caly*(sbly*sblz + cbly*cblz*sblx) + cblx*cblz*salx)
                - (cbcy*cbcz + sbcx*sbcy*sbcz)*(calx*caly*(cblz*sbly - cbly*sblx*sblz)
                - calx*saly*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sblz)
                + cbcx*sbcy*(salx*sblx + calx*caly*cblx*cbly + calx*cblx*saly*sbly);
   float crycrx = (cbcz*sbcy - cbcy*sbcx*sbcz)*(calx*caly*(cblz*sbly - cbly*sblx*sblz)
                - calx*saly*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sblz)
                - (sbcy*sbcz + cbcy*cbcz*sbcx)*(calx*saly*(cbly*sblz - cblz*sblx*sbly)
                - calx*caly*(sbly*sblz + cbly*cblz*sblx) + cblx*cblz*salx)
                + cbcx*cbcy*(salx*sblx + calx*caly*cblx*cbly + calx*cblx*saly*sbly);
   transformTobeMapped[1] = atan2(srycrx / cos(transformTobeMapped[0]),
                                  crycrx / cos(transformTobeMapped[0]));

   float srzcrx = sbcx*(cblx*cbly*(calz*saly - caly*salx*salz)
                - cblx*sbly*(caly*calz + salx*saly*salz) + calx*salz*sblx)
                - cbcx*cbcz*((caly*calz + salx*saly*salz)*(cbly*sblz - cblz*sblx*sbly)
                + (calz*saly - caly*salx*salz)*(sbly*sblz + cbly*cblz*sblx)
                - calx*cblx*cblz*salz) + cbcx*sbcz*((caly*calz + salx*saly*salz)*(cbly*cblz
                + sblx*sbly*sblz) + (calz*saly - caly*salx*salz)*(cblz*sbly - cbly*sblx*sblz)
                + calx*cblx*salz*sblz);
   float crzcrx = sbcx*(cblx*sbly*(caly*salz - calz*salx*saly)
                - cblx*cbly*(saly*salz + caly*calz*salx) + calx*calz*sblx)
                + cbcx*cbcz*((saly*salz + caly*calz*salx)*(sbly*sblz + cbly*cblz*sblx)
                + (caly*salz - calz*salx*saly)*(cbly*sblz - cblz*sblx*sbly)
                + calx*calz*cblx*cblz) - cbcx*sbcz*((saly*salz + caly*calz*salx)*(cblz*sbly
                - cbly*sblx*sblz) + (caly*salz - calz*salx*saly)*(cbly*cblz + sblx*sbly*sblz)
                - calx*calz*cblx*sblz);
   transformTobeMapped[2] = atan2(srzcrx / cos(transformTobeMapped[0]),
                                  crzcrx / cos(transformTobeMapped[0]));

   x1 = cos(transformTobeMapped[2]) * transformIncre[3] - sin(transformTobeMapped[2]) * transformIncre[4];
   y1 = sin(transformTobeMapped[2]) * transformIncre[3] + cos(transformTobeMapped[2]) * transformIncre[4];
   z1 = transformIncre[5];

   x2 = x1;
   y2 = cos(transformTobeMapped[0]) * y1 - sin(transformTobeMapped[0]) * z1;
   z2 = sin(transformTobeMapped[0]) * y1 + cos(transformTobeMapped[0]) * z1;

   transformTobeMapped[3] = transformAftMapped[3]
                          - (cos(transformTobeMapped[1]) * x2 + sin(transformTobeMapped[1]) * z2);
   transformTobeMapped[4] = transformAftMapped[4] - y2;
   transformTobeMapped[5] = transformAftMapped[5]
                          - (-sin(transformTobeMapped[1]) * x2 + cos(transformTobeMapped[1]) * z2);
}

void transformUpdate()
{
   if (imuPointerLast >= 0) {
     float imuRollLast = 0, imuPitchLast = 0;
     while (imuPointerFront != imuPointerLast) {
       if (timeLaserOdometry + scanPeriod < imuTime[imuPointerFront]) {
         break;
       }
       imuPointerFront = (imuPointerFront + 1) % imuQueLength;
     }

     if (timeLaserOdometry + scanPeriod > imuTime[imuPointerFront]) {
       imuRollLast = imuRoll[imuPointerFront];
       imuPitchLast = imuPitch[imuPointerFront];
     } else {
       int imuPointerBack = (imuPointerFront + imuQueLength - 1) % imuQueLength;
       float ratioFront = (timeLaserOdometry + scanPeriod - imuTime[imuPointerBack])
                        / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
       float ratioBack = (imuTime[imuPointerFront] - timeLaserOdometry - scanPeriod)
                       / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);

       imuRollLast = imuRoll[imuPointerFront] * ratioFront + imuRoll[imuPointerBack] * ratioBack;
       imuPitchLast = imuPitch[imuPointerFront] * ratioFront + imuPitch[imuPointerBack] * ratioBack;
     }

     transformTobeMapped[0] = 0.998 * transformTobeMapped[0] + 0.002 * imuPitchLast;
     transformTobeMapped[2] = 0.998 * transformTobeMapped[2] + 0.002 * imuRollLast;
   }

   //transformTobeMapped[0] = 0.998 * transformTobeMapped[0] + 0.002 * transformSum[0];
   //transformTobeMapped[2] = 0.998 * transformTobeMapped[2] + 0.002 * transformSum[2];

   for (int i = 0; i < 6; i++) {
     transformBefMapped[i] = transformSum[i];
     transformAftMapped[i] = transformTobeMapped[i];
   }
}

void pointAssociateToMap(pcl::PointXYZINormal *pi, pcl::PointXYZINormal *po)
{
   float x1 = cos(transformTobeMapped[2]) * pi->x
            - sin(transformTobeMapped[2]) * pi->y;
   float y1 = sin(transformTobeMapped[2]) * pi->x
            + cos(transformTobeMapped[2]) * pi->y;
   float z1 = pi->z;

   float x2 = x1;
   float y2 = cos(transformTobeMapped[0]) * y1 - sin(transformTobeMapped[0]) * z1;
   float z2 = sin(transformTobeMapped[0]) * y1 + cos(transformTobeMapped[0]) * z1;

   po->x = cos(transformTobeMapped[1]) * x2 + sin(transformTobeMapped[1]) * z2
         + transformTobeMapped[3];
   po->y = y2 + transformTobeMapped[4];
   po->z = -sin(transformTobeMapped[1]) * x2 + cos(transformTobeMapped[1]) * z2
         + transformTobeMapped[5];
   po->intensity = pi->intensity;
   po->curvature = pi->curvature;
}

void pointAssociateTobeMapped(pcl::PointXYZINormal *pi, pcl::PointXYZINormal *po)
{
   float x1 = cos(transformTobeMapped[1]) * (pi->x - transformTobeMapped[3])
            - sin(transformTobeMapped[1]) * (pi->z - transformTobeMapped[5]);
   float y1 = pi->y - transformTobeMapped[4];
   float z1 = sin(transformTobeMapped[1]) * (pi->x - transformTobeMapped[3])
            + cos(transformTobeMapped[1]) * (pi->z - transformTobeMapped[5]);

   float x2 = x1;
   float y2 = cos(transformTobeMapped[0]) * y1 + sin(transformTobeMapped[0]) * z1;
   float z2 = -sin(transformTobeMapped[0]) * y1 + cos(transformTobeMapped[0]) * z1;

   po->x = cos(transformTobeMapped[2]) * x2
         + sin(transformTobeMapped[2]) * y2;
   po->y = -sin(transformTobeMapped[2]) * x2
         + cos(transformTobeMapped[2]) * y2;
   po->z = z2;
   po->intensity = pi->intensity;
   po->curvature = pi->curvature;
}

void laserCloudCornerLastHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudCornerLast2)
{
   timeLaserCloudCornerLast = laserCloudCornerLast2->header.stamp.toSec();

   laserCloudCornerLast->clear();
   pcl::fromROSMsg(*laserCloudCornerLast2, *laserCloudCornerLast);

   newLaserCloudCornerLast = true;
}

void laserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudSurfLast2)
{
   timeLaserCloudSurfLast = laserCloudSurfLast2->header.stamp.toSec();

   laserCloudSurfLast->clear();
   pcl::fromROSMsg(*laserCloudSurfLast2, *laserCloudSurfLast);

   newLaserCloudSurfLast = true;
}

void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudFullRes2)
{
   timeLaserCloudFullRes = laserCloudFullRes2->header.stamp.toSec();

   laserCloudFullRes->clear();
   pcl::fromROSMsg(*laserCloudFullRes2, *laserCloudFullRes);

   newLaserCloudFullRes = true;
}

void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry)
{
   timeLaserOdometry = laserOdometry->header.stamp.toSec();

   double roll, pitch, yaw;
   geometry_msgs::Quaternion geoQuat = laserOdometry->pose.pose.orientation;
   tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

   transformSum[0] = -pitch;
   transformSum[1] = -yaw;
   transformSum[2] = roll;

   transformSum[3] = laserOdometry->pose.pose.position.x;
   transformSum[4] = laserOdometry->pose.pose.position.y;
   transformSum[5] = laserOdometry->pose.pose.position.z;

   newLaserOdometry = true;
}

void imuHandler(const sensor_msgs::Imu::ConstPtr& imuIn)
{
   double roll, pitch, yaw;
   tf::Quaternion orientation;
   tf::quaternionMsgToTF(imuIn->orientation, orientation);
   tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

   imuPointerLast = (imuPointerLast + 1) % imuQueLength;

   imuTime[imuPointerLast] = imuIn->header.stamp.toSec();
   imuRoll[imuPointerLast] = roll;
   imuPitch[imuPointerLast] = pitch;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "mapping");
	ros::NodeHandle nh;

	ros::Subscriber subLaserCloudCornerLast = nh.subscribe<sensor_msgs::PointCloud2>
											 ("/laser_cloud_corner_last", 2, laserCloudCornerLastHandler);

	ros::Subscriber subLaserCloudSurfLast = nh.subscribe<sensor_msgs::PointCloud2>
										   ("/laser_cloud_surf_last", 2, laserCloudSurfLastHandler);

	ros::Subscriber subLaserOdometry = nh.subscribe<nav_msgs::Odometry>
									  ("/laser_odom_to_init", 5, laserOdometryHandler);

	ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>
										  ("/velodyne_cloud_3", 2, laserCloudFullResHandler);

	ros::Subscriber subImu = nh.subscribe<sensor_msgs::Imu> ("/imu/data", 50, imuHandler);

	//ros::Publisher pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>
	//									  ("/laser_cloud_surround", 1);

	ros::Publisher pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>
										 ("/velodyne_cloud_registered", 2);

	//ros::Publisher pub1 = nh.advertise<sensor_msgs::PointCloud2> ("/pc1", 2);

	//ros::Publisher pub2 = nh.advertise<sensor_msgs::PointCloud2> ("/pc2", 2);

	//ros::Publisher pub1 = nh.advertise<sensor_msgs::PointCloud2> ("/pc3", 2);

	//ros::Publisher pub2 = nh.advertise<sensor_msgs::PointCloud2> ("/pc4", 2);

	ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry> ("/aft_mapped_to_init", 5);

	ros::ServiceServer srv_record_map = nh.advertiseService("srv_record_map", recordMapHandler);
	   
	nav_msgs::Odometry odomAftMapped;
	odomAftMapped.header.frame_id = "/camera_init";
	odomAftMapped.child_frame_id = "/aft_mapped";

	tf::TransformBroadcaster tfBroadcaster;
	tf::StampedTransform aftMappedTrans;
	aftMappedTrans.frame_id_ = "/camera_init";
	aftMappedTrans.child_frame_id_ = "/aft_mapped";

	std::vector<int> pointSearchInd;
	std::vector<float> pointSearchSqDis;

	pcl::PointXYZINormal pointOri, pointSel, pointProj, coeff;

	cv::Mat matA0(5, 3, CV_32F, cv::Scalar::all(0));
	cv::Mat matB0(5, 1, CV_32F, cv::Scalar::all(-1));
	cv::Mat matX0(3, 1, CV_32F, cv::Scalar::all(0));

	cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0));
	cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0));
	cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));

	bool isDegenerate = false;
	cv::Mat matP(6, 6, CV_32F, cv::Scalar::all(0));

	pcl::VoxelGrid<pcl::PointXYZINormal> downSizeFilterCorner;
	downSizeFilterCorner.setLeafSize(0.2, 0.2, 0.2);

	pcl::VoxelGrid<pcl::PointXYZINormal> downSizeFilterSurf;
	downSizeFilterSurf.setLeafSize(0.4, 0.4, 0.4);

	pcl::VoxelGrid<pcl::PointXYZINormal> downSizeFilterMap;
	downSizeFilterMap.setLeafSize(0.6, 0.6, 0.6);

	for (int i = 0; i < laserCloudNum; i++) 
	{
		laserCloudCornerArray[i].reset(new pcl::PointCloud<pcl::PointXYZINormal>());
		laserCloudSurfArray[i].reset(new pcl::PointCloud<pcl::PointXYZINormal>());
		laserCloudCornerArray2[i].reset(new pcl::PointCloud<pcl::PointXYZINormal>());
		laserCloudSurfArray2[i].reset(new pcl::PointCloud<pcl::PointXYZINormal>());
	}

	int frameCount = stackFrameNum - 1;
	int mapFrameCount = mapFrameNum - 1;
	ros::Rate rate(100);
	//bool status = ros::ok();
	while (ros::ok())
	{
		//ros::spinOnce();
		if (newLaserCloudCornerLast && newLaserCloudSurfLast && newLaserCloudFullRes && newLaserOdometry &&
			fabs(timeLaserCloudCornerLast - timeLaserOdometry) < 0.005 &&
			fabs(timeLaserCloudSurfLast - timeLaserOdometry) < 0.005 &&
			fabs(timeLaserCloudFullRes - timeLaserOdometry) < 0.005) 
		{
			newLaserCloudCornerLast = false;
			newLaserCloudSurfLast = false;
			newLaserCloudFullRes = false;
			newLaserOdometry = false;

			frameCount++;
			if (frameCount >= stackFrameNum)
			{
				transformAssociateToMap();
				int laserCloudCornerLastNum = laserCloudCornerLast->points.size();
				for (int i = 0; i < laserCloudCornerLastNum; i++) 
				{
					pointAssociateToMap(&laserCloudCornerLast->points[i], &pointSel);
					laserCloudCornerStack2->push_back(pointSel);
				}
				int laserCloudSurfLastNum = laserCloudSurfLast->points.size();
				for (int i = 0; i < laserCloudSurfLastNum; i++)
				{
					pointAssociateToMap(&laserCloudSurfLast->points[i], &pointSel);
					laserCloudSurfStack2->push_back(pointSel);
				}
			}

			if (frameCount >= stackFrameNum)
			{
				frameCount = 0;

				//用来测试全局地图上的点，是否在当前视角内
				pcl::PointXYZINormal pointOnYAxis;
				pointOnYAxis.x = 0.0;
				pointOnYAxis.y = 10.0;  //
				pointOnYAxis.z = 0.0;
				pointAssociateToMap(&pointOnYAxis, &pointOnYAxis);

				//设备位于哪个栅格（归到一象限）
				int centerCubeI = int((transformTobeMapped[3] + voxel_range/2.0) / voxel_range) + laserCloudCenWidth;
				int centerCubeJ = int((transformTobeMapped[4] + voxel_range/2.0) / voxel_range) + laserCloudCenHeight;
				int centerCubeK = int((transformTobeMapped[5] + voxel_range/2.0) / voxel_range) + laserCloudCenDepth;
				if ((transformTobeMapped[3] + voxel_range/2.0) < 0) centerCubeI--;
				if ((transformTobeMapped[4] + voxel_range/2.0) < 0) centerCubeJ--;
				if ((transformTobeMapped[5] + voxel_range/2.0) < 0) centerCubeK--;

				
				//下面6个while，即当前设备位置，到达区域边界了，整个区域要平移一格
				//始终保持距离当前位置有3个栅格的数据
				//即 局部的三维空间数组大小不变，变的其实是laserCloudCenWidth .. 这三个参数，它仨才是真正相对于全局的
				while (centerCubeI < 3)
				{
					for (int j = 0; j < laserCloudHeight; j++)
					{
						for (int k = 0; k < laserCloudDepth; k++)
						{
							int i = laserCloudWidth - 1;
							pcl::PointCloud<pcl::PointXYZINormal>::Ptr laserCloudCubeCornerPointer =
									laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
							pcl::PointCloud<pcl::PointXYZINormal>::Ptr laserCloudCubeSurfPointer =
									laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
							for (; i >= 1; i--)
							{
								laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
										laserCloudCornerArray[i - 1 + laserCloudWidth*j + laserCloudWidth * laserCloudHeight * k];
								laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
										laserCloudSurfArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
							}
							
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = laserCloudCubeCornerPointer;
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = laserCloudCubeSurfPointer;
							laserCloudCubeCornerPointer->clear();
							laserCloudCubeSurfPointer->clear();
						}
					}
					centerCubeI++;
					laserCloudCenWidth++;
				}

				while (centerCubeI >= laserCloudWidth - 3)
				{
					for (int j = 0; j < laserCloudHeight; j++)
					{
						for (int k = 0; k < laserCloudDepth; k++)
						{
							int i = 0;
							pcl::PointCloud<pcl::PointXYZINormal>::Ptr laserCloudCubeCornerPointer =
									laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
							pcl::PointCloud<pcl::PointXYZINormal>::Ptr laserCloudCubeSurfPointer =
									laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
							for (; i < laserCloudWidth - 1; i++)
							{
								laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
										laserCloudCornerArray[i + 1 + laserCloudWidth*j + laserCloudWidth * laserCloudHeight * k];
								laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
										laserCloudSurfArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
							}
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = laserCloudCubeCornerPointer;
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = laserCloudCubeSurfPointer;
							laserCloudCubeCornerPointer->clear();
							laserCloudCubeSurfPointer->clear();
						}
					}

					centerCubeI--;
					laserCloudCenWidth--;
				}

				while (centerCubeJ < 3)
				{
					for (int i = 0; i < laserCloudWidth; i++)
					{
						for (int k = 0; k < laserCloudDepth; k++)
						{
							int j = laserCloudHeight - 1;
							pcl::PointCloud<pcl::PointXYZINormal>::Ptr laserCloudCubeCornerPointer =
									laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
							pcl::PointCloud<pcl::PointXYZINormal>::Ptr laserCloudCubeSurfPointer =
									laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
							for (; j >= 1; j--)
							{
								laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
										laserCloudCornerArray[i + laserCloudWidth*(j - 1) + laserCloudWidth * laserCloudHeight*k];
								laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
										laserCloudSurfArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight*k];
							}
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = laserCloudCubeCornerPointer;
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = laserCloudCubeSurfPointer;
							laserCloudCubeCornerPointer->clear();
							laserCloudCubeSurfPointer->clear();
						}
					}

					centerCubeJ++;
					laserCloudCenHeight++;
				}

				while (centerCubeJ >= laserCloudHeight - 3)
				{
					for (int i = 0; i < laserCloudWidth; i++)
					{
						for (int k = 0; k < laserCloudDepth; k++)
						{
							int j = 0;
							pcl::PointCloud<pcl::PointXYZINormal>::Ptr laserCloudCubeCornerPointer =
									laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
							pcl::PointCloud<pcl::PointXYZINormal>::Ptr laserCloudCubeSurfPointer =
									laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
							for (; j < laserCloudHeight - 1; j++)
							{
								laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
										laserCloudCornerArray[i + laserCloudWidth*(j + 1) + laserCloudWidth * laserCloudHeight*k];
								laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
										laserCloudSurfArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight*k];
							}
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = laserCloudCubeCornerPointer;
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = laserCloudCubeSurfPointer;
							laserCloudCubeCornerPointer->clear();
							laserCloudCubeSurfPointer->clear();
						}
					}

					centerCubeJ--;
					laserCloudCenHeight--;
				}

				while (centerCubeK < 3)
				{
					for (int i = 0; i < laserCloudWidth; i++)
					{
						for (int j = 0; j < laserCloudHeight; j++)
						{
							int k = laserCloudDepth - 1;
							pcl::PointCloud<pcl::PointXYZINormal>::Ptr laserCloudCubeCornerPointer =
									laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
							pcl::PointCloud<pcl::PointXYZINormal>::Ptr laserCloudCubeSurfPointer =
									laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
							for (; k >= 1; k--)
							{
								laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
										laserCloudCornerArray[i + laserCloudWidth*j + laserCloudWidth * laserCloudHeight*(k - 1)];
								laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
										laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight*(k - 1)];
							}
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = laserCloudCubeCornerPointer;
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = laserCloudCubeSurfPointer;
							laserCloudCubeCornerPointer->clear();
							laserCloudCubeSurfPointer->clear();
						}
					}

					centerCubeK++;
					laserCloudCenDepth++;
				}

				while (centerCubeK >= laserCloudDepth - 3)
				{
					for (int i = 0; i < laserCloudWidth; i++)
					{
						for (int j = 0; j < laserCloudHeight; j++)
						{
							int k = 0;
							pcl::PointCloud<pcl::PointXYZINormal>::Ptr laserCloudCubeCornerPointer =
									laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
							pcl::PointCloud<pcl::PointXYZINormal>::Ptr laserCloudCubeSurfPointer =
									laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
							for (; k < laserCloudDepth - 1; k++) 
							{
								laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
										laserCloudCornerArray[i + laserCloudWidth*j + laserCloudWidth * laserCloudHeight*(k + 1)];
								laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
										laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight*(k + 1)];
							}
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = laserCloudCubeCornerPointer;
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = laserCloudCubeSurfPointer;
							laserCloudCubeCornerPointer->clear();
							laserCloudCubeSurfPointer->clear();
						}
					}

					centerCubeK--;
					laserCloudCenDepth--;
				}

				
				//下面从历史全局地图上，寻找当前位置周围2+1+2个栅格内的所有点
				int laserCloudValidNum = 0;
				int laserCloudSurroundNum = 0;
				float half_voxel_range = voxel_range / 2.0;
				for (int i = centerCubeI - 2; i <= centerCubeI + 2; i++)
				{
					for (int j = centerCubeJ - 2; j <= centerCubeJ + 2; j++)
					{
						for (int k = centerCubeK - 2; k <= centerCubeK + 2; k++)
						{
							if (i >= 0 && i < laserCloudWidth &&
									j >= 0 && j < laserCloudHeight &&
									k >= 0 && k < laserCloudDepth)
							{

								float centerX = voxel_range * (i - laserCloudCenWidth);
								float centerY = voxel_range * (j - laserCloudCenHeight);
								float centerZ = voxel_range * (k - laserCloudCenDepth);

								bool isInLaserFOV = false;
								//多向外扩展半个栅格来检验
								for (int ii = -1; ii <= 1; ii += 2)
								{
									for (int jj = -1; jj <= 1; jj += 2)
									{
										for (int kk = -1; kk <= 1; kk += 2)
										{
											float cornerX = centerX + half_voxel_range * ii;
											float cornerY = centerY + half_voxel_range * jj;
											float cornerZ = centerZ + half_voxel_range * kk;

											float squaredSide1 = (transformTobeMapped[3] - cornerX)
																		  * (transformTobeMapped[3] - cornerX)
																		  + (transformTobeMapped[4] - cornerY)
																		  * (transformTobeMapped[4] - cornerY)
																		  + (transformTobeMapped[5] - cornerZ)
																		  * (transformTobeMapped[5] - cornerZ);

											float squaredSide2 = (pointOnYAxis.x - cornerX) * (pointOnYAxis.x - cornerX)
															  + (pointOnYAxis.y - cornerY) * (pointOnYAxis.y - cornerY)
															  + (pointOnYAxis.z - cornerZ) * (pointOnYAxis.z - cornerZ);

											//保证所获取的点在当前视角的正负30度之间，之外的点用不到（a*a+b*b-c*c = 2*a*b*cos(theta)）
											float check1 = 100.0 + squaredSide1 - squaredSide2
														- 17.32 * sqrt(squaredSide1);

											float check2 = 100.0 + squaredSide1 - squaredSide2
														+ 17.32 * sqrt(squaredSide1);

											if (check1 < 0 && check2 > 0) {
												isInLaserFOV = true;
											}
										}
									}
								}

								if (isInLaserFOV) 
								{
									//有效点的index存下来
									laserCloudValidInd[laserCloudValidNum] = i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
									laserCloudValidNum++;
								}
								laserCloudSurroundInd[laserCloudSurroundNum] = i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
								laserCloudSurroundNum++;
							}
						}
					}
				}

				
				
				laserCloudCornerFromMap->clear();
				laserCloudSurfFromMap->clear();
				for (int i = 0; i < laserCloudValidNum; i++) {
					*laserCloudCornerFromMap += *laserCloudCornerArray[laserCloudValidInd[i]];
					*laserCloudSurfFromMap += *laserCloudSurfArray[laserCloudValidInd[i]];
				}
				int laserCloudCornerFromMapNum = laserCloudCornerFromMap->points.size();
				int laserCloudSurfFromMapNum = laserCloudSurfFromMap->points.size();

				
				//当前特征点降采样一下
				int laserCloudCornerStackNum2 = laserCloudCornerStack2->points.size();
				for (int i = 0; i < laserCloudCornerStackNum2; i++) {
					pointAssociateTobeMapped(&laserCloudCornerStack2->points[i], &laserCloudCornerStack2->points[i]);
				}

				int laserCloudSurfStackNum2 = laserCloudSurfStack2->points.size();
				for (int i = 0; i < laserCloudSurfStackNum2; i++) {
					pointAssociateTobeMapped(&laserCloudSurfStack2->points[i], &laserCloudSurfStack2->points[i]);
				}

				laserCloudCornerStack->clear();
				downSizeFilterCorner.setInputCloud(laserCloudCornerStack2);
				downSizeFilterCorner.filter(*laserCloudCornerStack);
				int laserCloudCornerStackNum = laserCloudCornerStack->points.size();

				laserCloudSurfStack->clear();
				downSizeFilterSurf.setInputCloud(laserCloudSurfStack2);
				downSizeFilterSurf.filter(*laserCloudSurfStack);
				int laserCloudSurfStackNum = laserCloudSurfStack->points.size();

				laserCloudCornerStack2->clear();
				laserCloudSurfStack2->clear();
				
				
				//下面真正开始求解
				if (laserCloudCornerFromMapNum > 10 && laserCloudSurfFromMapNum > 100)
				{
					kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMap);
					kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMap);

					//开始迭代啦
					for (int iterCount = 0; iterCount < 30; iterCount++)
					{
						laserCloudOri->clear();
						//laserCloudSel->clear();
						//laserCloudCorr->clear();
						//laserCloudProj->clear();
						coeffSel->clear();

						for (int i = 0; i < laserCloudCornerStackNum; i++)
						{
							pointOri = laserCloudCornerStack->points[i];
							pointAssociateToMap(&pointOri, &pointSel);
							kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

							if (pointSearchSqDis[4] < 1.0)
							{
								float cx = 0;
								float cy = 0;
								float cz = 0;
								for (int j = 0; j < 5; j++)
								{
									cx += laserCloudCornerFromMap->points[pointSearchInd[j]].x;
									cy += laserCloudCornerFromMap->points[pointSearchInd[j]].y;
									cz += laserCloudCornerFromMap->points[pointSearchInd[j]].z;
								}
								cx /= 5;
								cy /= 5;
								cz /= 5;

								float a11 = 0;
								float a12 = 0;
								float a13 = 0;
								float a22 = 0;
								float a23 = 0;
								float a33 = 0;
								for (int j = 0; j < 5; j++)
								{
									float ax = laserCloudCornerFromMap->points[pointSearchInd[j]].x - cx;
									float ay = laserCloudCornerFromMap->points[pointSearchInd[j]].y - cy;
									float az = laserCloudCornerFromMap->points[pointSearchInd[j]].z - cz;

									a11 += ax * ax;
									a12 += ax * ay;
									a13 += ax * az;
									a22 += ay * ay;
									a23 += ay * az;
									a33 += az * az;
								}
								a11 /= 5;
								a12 /= 5;
								a13 /= 5;
								a22 /= 5;
								a23 /= 5;
								a33 /= 5;
								//构造协方差矩阵
								matA1.at<float>(0, 0) = a11;
								matA1.at<float>(0, 1) = a12;
								matA1.at<float>(0, 2) = a13;
								matA1.at<float>(1, 0) = a12;
								matA1.at<float>(1, 1) = a22;
								matA1.at<float>(1, 2) = a23;
								matA1.at<float>(2, 0) = a13;
								matA1.at<float>(2, 1) = a23;
								matA1.at<float>(2, 2) = a33;
								//特征值和特征向量
								cv::eigen(matA1, matD1, matV1);

								if (matD1.at<float>(0, 0) > 3 * matD1.at<float>(0, 1))
								{
									float x0 = pointSel.x;
									float y0 = pointSel.y;
									float z0 = pointSel.z;
									float x1 = cx + 0.1 * matV1.at<float>(0, 0);
									float y1 = cy + 0.1 * matV1.at<float>(0, 1);
									float z1 = cz + 0.1 * matV1.at<float>(0, 2);
									float x2 = cx - 0.1 * matV1.at<float>(0, 0);
									float y2 = cy - 0.1 * matV1.at<float>(0, 1);
									float z2 = cz - 0.1 * matV1.at<float>(0, 2);

									float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
										  * ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
										  + ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
										  * ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
										  + ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))
										  * ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));

									float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

									float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
										+ (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;

									float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
										- (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

									float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
										+ (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

									float ld2 = a012 / l12;

									pointProj = pointSel;
									pointProj.x -= la * ld2;
									pointProj.y -= lb * ld2;
									pointProj.z -= lc * ld2;

									float s = 1 - 0.9 * fabs(ld2);

									coeff.x = s * la;
									coeff.y = s * lb;
									coeff.z = s * lc;
									coeff.intensity = s * ld2;

									if (s > 0.1)
									{
										laserCloudOri->push_back(pointOri);
										//laserCloudSel->push_back(pointSel);
										//laserCloudProj->push_back(pointProj);
										//laserCloudCorr->push_back(laserCloudCornerFromMap->points[pointSearchInd[0]]);
										//laserCloudCorr->push_back(laserCloudCornerFromMap->points[pointSearchInd[1]]);
										//laserCloudCorr->push_back(laserCloudCornerFromMap->points[pointSearchInd[2]]);
										//laserCloudCorr->push_back(laserCloudCornerFromMap->points[pointSearchInd[3]]);
										//laserCloudCorr->push_back(laserCloudCornerFromMap->points[pointSearchInd[4]]);
										coeffSel->push_back(coeff);
									}
								}
							}
						}

						for (int i = 0; i < laserCloudSurfStackNum; i++)
						{
							pointOri = laserCloudSurfStack->points[i];
							pointAssociateToMap(&pointOri, &pointSel);
							kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

							if (pointSearchSqDis[4] < 1.0)
							{
								for (int j = 0; j < 5; j++)
								{
									matA0.at<float>(j, 0) = laserCloudSurfFromMap->points[pointSearchInd[j]].x;
									matA0.at<float>(j, 1) = laserCloudSurfFromMap->points[pointSearchInd[j]].y;
									matA0.at<float>(j, 2) = laserCloudSurfFromMap->points[pointSearchInd[j]].z;
								}
								cv::solve(matA0, matB0, matX0, cv::DECOMP_QR);

								float pa = matX0.at<float>(0, 0);
								float pb = matX0.at<float>(1, 0);
								float pc = matX0.at<float>(2, 0);
								float pd = 1;

								float ps = sqrt(pa * pa + pb * pb + pc * pc);
								pa /= ps;
								pb /= ps;
								pc /= ps;
								pd /= ps;

								bool planeValid = true;
								for (int j = 0; j < 5; j++)
								{
									if (fabs(pa * laserCloudSurfFromMap->points[pointSearchInd[j]].x +
											pb * laserCloudSurfFromMap->points[pointSearchInd[j]].y +
											pc * laserCloudSurfFromMap->points[pointSearchInd[j]].z + pd) > 0.2)
									{
										planeValid = false;
										break;
									}
								}

								if (planeValid)
								{
									float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

									pointProj = pointSel;
									pointProj.x -= pa * pd2;
									pointProj.y -= pb * pd2;
									pointProj.z -= pc * pd2;

									float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointSel.x * pointSel.x
										   + pointSel.y * pointSel.y + pointSel.z * pointSel.z));

									coeff.x = s * pa;
									coeff.y = s * pb;
									coeff.z = s * pc;
									coeff.intensity = s * pd2;

									if (s > 0.1)
									{
										laserCloudOri->push_back(pointOri);
										//laserCloudSel->push_back(pointSel);
										//laserCloudProj->push_back(pointProj);
										//laserCloudCorr->push_back(laserCloudSurfFromMap->points[pointSearchInd[0]]);
										//laserCloudCorr->push_back(laserCloudSurfFromMap->points[pointSearchInd[1]]);
										//laserCloudCorr->push_back(laserCloudSurfFromMap->points[pointSearchInd[2]]);
										//laserCloudCorr->push_back(laserCloudSurfFromMap->points[pointSearchInd[3]]);
										//laserCloudCorr->push_back(laserCloudSurfFromMap->points[pointSearchInd[4]]);
										coeffSel->push_back(coeff);
									}
								}
							}
						}

						float srx = sin(transformTobeMapped[0]);
						float crx = cos(transformTobeMapped[0]);
						float sry = sin(transformTobeMapped[1]);
						float cry = cos(transformTobeMapped[1]);
						float srz = sin(transformTobeMapped[2]);
						float crz = cos(transformTobeMapped[2]);

						int laserCloudSelNum = laserCloudOri->points.size();
						if (laserCloudSelNum < 50) {
							continue;
						}

						cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
						cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
						cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
						cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
						cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
						cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));
						for (int i = 0; i < laserCloudSelNum; i++)
						{
							pointOri = laserCloudOri->points[i];
							coeff = coeffSel->points[i];

							float arx = (crx*sry*srz*pointOri.x + crx*crz*sry*pointOri.y - srx*sry*pointOri.z) * coeff.x
									 + (-srx*srz*pointOri.x - crz*srx*pointOri.y - crx*pointOri.z) * coeff.y
									 + (crx*cry*srz*pointOri.x + crx*cry*crz*pointOri.y - cry*srx*pointOri.z) * coeff.z;

							float ary = ((cry*srx*srz - crz*sry)*pointOri.x
									 + (sry*srz + cry*crz*srx)*pointOri.y + crx*cry*pointOri.z) * coeff.x
									 + ((-cry*crz - srx*sry*srz)*pointOri.x
									 + (cry*srz - crz*srx*sry)*pointOri.y - crx*sry*pointOri.z) * coeff.z;

							float arz = ((crz*srx*sry - cry*srz)*pointOri.x + (-cry*crz-srx*sry*srz)*pointOri.y)*coeff.x
									 + (crx*crz*pointOri.x - crx*srz*pointOri.y) * coeff.y
									 + ((sry*srz + cry*crz*srx)*pointOri.x + (crz*sry-cry*srx*srz)*pointOri.y)*coeff.z;

							matA.at<float>(i, 0) = arx;
							matA.at<float>(i, 1) = ary;
							matA.at<float>(i, 2) = arz;
							matA.at<float>(i, 3) = coeff.x;
							matA.at<float>(i, 4) = coeff.y;
							matA.at<float>(i, 5) = coeff.z;
							matB.at<float>(i, 0) = -coeff.intensity;
						}
						cv::transpose(matA, matAt);
						matAtA = matAt * matA;
						matAtB = matAt * matB;
						cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

						if (iterCount == 0)
						{
							cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
							cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
							cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

						   cv::eigen(matAtA, matE, matV);
						   matV.copyTo(matV2);

						   isDegenerate = false;
						   float eignThre[6] = {100, 100, 100, 100, 100, 100};
							for (int i = 5; i >= 0; i--)
							{
								if (matE.at<float>(0, i) < eignThre[i])
								{
									for (int j = 0; j < 6; j++) {
										matV2.at<float>(i, j) = 0;
									}
									isDegenerate = true;
								}
								else {
									break;
								}
							}
								matP = matV.inv() * matV2;
						}

						if (isDegenerate /*&& 0*/)
						{
							cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
							matX.copyTo(matX2);
							matX = matP * matX2;
							//ROS_INFO ("laser mapping degenerate");
						}

						transformTobeMapped[0] += matX.at<float>(0, 0);
						transformTobeMapped[1] += matX.at<float>(1, 0);
						transformTobeMapped[2] += matX.at<float>(2, 0);
						transformTobeMapped[3] += matX.at<float>(3, 0);
						transformTobeMapped[4] += matX.at<float>(4, 0);
						transformTobeMapped[5] += matX.at<float>(5, 0);

						float deltaR = sqrt(matX.at<float>(0, 0) * 180 / PI * matX.at<float>(0, 0) * 180 / PI
								  + matX.at<float>(1, 0) * 180 / PI * matX.at<float>(1, 0) * 180 / PI
								  + matX.at<float>(2, 0) * 180 / PI * matX.at<float>(2, 0) * 180 / PI);
						float deltaT = sqrt(matX.at<float>(3, 0) * 100 * matX.at<float>(3, 0) * 100
								  + matX.at<float>(4, 0) * 100 * matX.at<float>(4, 0) * 100
								  + matX.at<float>(5, 0) * 100 * matX.at<float>(5, 0) * 100);

						if (deltaR < 0.05 && deltaT < 0.05) {
							break;
						}

						//ROS_INFO ("iter: %d, deltaR: %f, deltaT: %f", iterCount, deltaR, deltaT);
					}//。。次迭代的结束

					transformUpdate();
				}//特征点数量的判断结束

           
				//角点转换为全局坐标，并放入全局数组
				for (int i = 0; i < laserCloudCornerStackNum; i++)
				{
					pointAssociateToMap(&laserCloudCornerStack->points[i], &pointSel);

					int cubeI = int((pointSel.x + half_voxel_range) / voxel_range) + laserCloudCenWidth;
					int cubeJ = int((pointSel.y + half_voxel_range) / voxel_range) + laserCloudCenHeight;
					int cubeK = int((pointSel.z + half_voxel_range) / voxel_range) + laserCloudCenDepth;
					//ROS_INFO("cubeI:%d,cubeJ:%d,cubeK:%d",cubeI,cubeJ,cubeK);
					if (pointSel.x + half_voxel_range < 0) cubeI--;
					if (pointSel.y + half_voxel_range < 0) cubeJ--;
					if (pointSel.z + half_voxel_range < 0) cubeK--;
					//ROS_INFO ("pointSel.x: %f, pointSel.y: %f, pointSel.z: %f", pointSel.x, pointSel.y, pointSel.z);
					if (cubeI >= 0 && cubeI < laserCloudWidth &&
						cubeJ >= 0 && cubeJ < laserCloudHeight &&
						cubeK >= 0 && cubeK < laserCloudDepth)
					{
						int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
						//ROS_INFO ("cubeInd: %d", cubeInd);
						laserCloudCornerArray[cubeInd]->push_back(pointSel);
						
						if(is_record_map)
						{
							save_corner_point_for_map(pointSel);
						}
					}
				}
         
				//面形点转换为全局坐标，并放入全局数组
				for (int i = 0; i < laserCloudSurfStackNum; i++)
				{
					pointAssociateToMap(&laserCloudSurfStack->points[i], &pointSel);

					int cubeI = int((pointSel.x + half_voxel_range) / voxel_range) + laserCloudCenWidth;
					int cubeJ = int((pointSel.y + half_voxel_range) / voxel_range) + laserCloudCenHeight;
					int cubeK = int((pointSel.z + half_voxel_range) / voxel_range) + laserCloudCenDepth;

					if (pointSel.x + half_voxel_range < 0) cubeI--;
					if (pointSel.y + half_voxel_range < 0) cubeJ--;
					if (pointSel.z + half_voxel_range < 0) cubeK--;

					if (cubeI >= 0 && cubeI < laserCloudWidth &&
						cubeJ >= 0 && cubeJ < laserCloudHeight &&
						cubeK >= 0 && cubeK < laserCloudDepth) 
					{
						int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
						laserCloudSurfArray[cubeInd]->push_back(pointSel);
					
						if(is_record_map)
						{
							save_surf_point_for_map(pointSel);
						}
					}
				}
	   
				//全局数组中的点，由于加入了新点，重新降采样一下
				for (int i = 0; i < laserCloudValidNum; i++)
				{
					int ind = laserCloudValidInd[i];

					laserCloudCornerArray2[ind]->clear();
					downSizeFilterCorner.setInputCloud(laserCloudCornerArray[ind]);
					downSizeFilterCorner.filter(*laserCloudCornerArray2[ind]);

					laserCloudSurfArray2[ind]->clear();
					downSizeFilterSurf.setInputCloud(laserCloudSurfArray[ind]);
					downSizeFilterSurf.filter(*laserCloudSurfArray2[ind]);

					pcl::PointCloud<pcl::PointXYZINormal>::Ptr laserCloudTemp = laserCloudCornerArray[ind];
					laserCloudCornerArray[ind] = laserCloudCornerArray2[ind];
					laserCloudCornerArray2[ind] = laserCloudTemp;

					laserCloudTemp = laserCloudSurfArray[ind];
					laserCloudSurfArray[ind] = laserCloudSurfArray2[ind];
					laserCloudSurfArray2[ind] = laserCloudTemp;
				}

				//将当前位置提取出的一定范围内的历史map上的点发送出去
				/*mapFrameCount++;
				if (mapFrameCount >= mapFrameNum)
				{
					mapFrameCount = 0;

					laserCloudSurround2->clear();
					for (int i = 0; i < laserCloudSurroundNum; i++){
						int ind = laserCloudSurroundInd[i];
						*laserCloudSurround2 += *laserCloudCornerArray[ind];
						*laserCloudSurround2 += *laserCloudSurfArray[ind];
					}

					laserCloudSurround->clear();
					downSizeFilterCorner.setInputCloud(laserCloudSurround2);
					downSizeFilterCorner.filter(*laserCloudSurround);

					sensor_msgs::PointCloud2 laserCloudSurround3;
					pcl::toROSMsg(*laserCloudSurround, laserCloudSurround3);
					laserCloudSurround3.header.stamp = ros::Time().fromSec(timeLaserOdometry);
					laserCloudSurround3.header.frame_id = "/camera_init";
					pubLaserCloudSurround.publish(laserCloudSurround3);
				}*/

				
				//点云转到全局，然后发送出去
				int laserCloudFullResNum = laserCloudFullRes->points.size();
				for (int i = 0; i < laserCloudFullResNum; i++) {
					pointAssociateToMap(&laserCloudFullRes->points[i], &laserCloudFullRes->points[i]);
				}
				sensor_msgs::PointCloud2 laserCloudFullRes3;
				pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);
				laserCloudFullRes3.header.stamp = ros::Time().fromSec(timeLaserOdometry);
				laserCloudFullRes3.header.frame_id = "/camera_init";
				pubLaserCloudFullRes.publish(laserCloudFullRes3);

				//位姿发送出去
				geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw
									   (transformAftMapped[2], -transformAftMapped[0], -transformAftMapped[1]);

				odomAftMapped.header.stamp = ros::Time().fromSec(timeLaserOdometry);
				odomAftMapped.pose.pose.orientation.x = -geoQuat.y;
				odomAftMapped.pose.pose.orientation.y = -geoQuat.z;
				odomAftMapped.pose.pose.orientation.z = geoQuat.x;
				odomAftMapped.pose.pose.orientation.w = geoQuat.w;
				odomAftMapped.pose.pose.position.x = transformAftMapped[3];
				odomAftMapped.pose.pose.position.y = transformAftMapped[4];
				odomAftMapped.pose.pose.position.z = transformAftMapped[5];
				odomAftMapped.twist.twist.angular.x = transformBefMapped[0];
				odomAftMapped.twist.twist.angular.y = transformBefMapped[1];
				odomAftMapped.twist.twist.angular.z = transformBefMapped[2];
				odomAftMapped.twist.twist.linear.x = transformBefMapped[3];
				odomAftMapped.twist.twist.linear.y = transformBefMapped[4];
				odomAftMapped.twist.twist.linear.z = transformBefMapped[5];
				pubOdomAftMapped.publish(odomAftMapped);

				aftMappedTrans.stamp_ = ros::Time().fromSec(timeLaserOdometry);
				aftMappedTrans.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
				aftMappedTrans.setOrigin(tf::Vector3(transformAftMapped[3],
												  transformAftMapped[4], transformAftMapped[5]));
				tfBroadcaster.sendTransform(aftMappedTrans);

				/*sensor_msgs::PointCloud2 pc12;
				pcl::toROSMsg(*laserCloudCornerStack, pc12);
				pc12.header.stamp = ros::Time().fromSec(timeLaserOdometry);
				pc12.header.frame_id = "/camera";
				pub1.publish(pc12);

				sensor_msgs::PointCloud2 pc22;
				pcl::toROSMsg(*laserCloudSurfStack, pc22);
				pc22.header.stamp = ros::Time().fromSec(timeLaserOdometry);
				pc22.header.frame_id = "/camera";
				pub2.publish(pc22);

				sensor_msgs::PointCloud2 pc32;
				pcl::toROSMsg(*laserCloudSel, pc32);
				pc32.header.stamp = ros::Time().fromSec(timeLaserOdometry);
				pc32.header.frame_id = "/camera";
				pub3.publish(pc32);

				sensor_msgs::PointCloud2 pc42;
				pcl::toROSMsg(*laserCloudProj, pc42);
				pc42.header.stamp = ros::Time().fromSec(timeLaserOdometry);
				pc42.header.frame_id = "/camera";
				pub4.publish(pc42);*/
			}//if(frameCount >= stackFrameNum)的结束
		}//新点进来和时间间隔满足条件：判断的括号结束

		ros::spinOnce();
		//status = ros::ok();
		rate.sleep();

	}//while循环结束

	return 0;
}              
