#ifndef LOCAL_MAP_MAP_BUILDER_H
#define LOCAL_MAP_MAP_BUILDER_H

#include <vector>
#include <math.h>
#include <set>
#include <sstream>
#include <fstream>
#include <string>
#include <stdlib.h>  
#include <stdio.h>  

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/thread/mutex.hpp>

#include "parameter.h"
#include "map_ray_caster.h"

namespace local_map 
{
	class MapBuilder
	{

	public:
		/*构造&析构函数*/
		MapBuilder(unsigned short width, unsigned short height, double resolution);
		~MapBuilder();

	public:
		//通过激光点投射返回像素列表
		bool getRayCastToObstacle( double angle, double angle_resolution, double range, std::vector<size_t> &raycast);
		/*程序入口:更新地图，在main.cpp激光回调函数中调用*/
		void updateMap(sensor_msgs::LaserScan scan_in);
		//void updateMap(const sensor_msgs::LaserScanConstPtr &scan_in);
		/*程序入口:更新位姿，在main.cpp激光回调函数中调用*/
		void setPoseAndHeading(double x, double y, double z, double angle);
		/*程序出口:输出地图，在main.cpp激光回调函数中调用*/
		nav_msgs::OccupancyGrid getMap();

	private:

		
		//当前机器人的位置
		float now_x;
		float now_y;
		float now_z;
		float heading;
		//std::vector<GridData> mapData;			//地图中每个栅格的特征，其他函数对此变量进行处理
		nav_msgs::OccupancyGrid map_;			//输出的地图变量
		//std::vector<GridData> history_mapData;	//栅格的特征
		/*根据栅格特征mapData，生成新的栅格地图map_*/
		//void generate_map();
		float *frontMapData;

	private:
		Map_info mapinfo;//地图参数
		
	};
}

#endif  // LOCAL_MAP_MAP_BUILDER_H
