#include <local_map/map_builder.h>
#include <pthread.h>
#include <iostream>
boost::mutex  mtx_pose,mtx_map;

namespace local_map {

MapBuilder::MapBuilder(unsigned short width, unsigned short height, double resolution)
{
    map_.header.frame_id = "/camera_init";		//Rviz显示的frame_id
    map_.info.width = width;            //宽度方向上栅格地图个数
    map_.info.height = height;          //高度方向上栅格地图个数
    map_.info.resolution = resolution;  //栅格边长
    map_.data.assign(width * height, labels[0]);    //初始化栅格地图标签为0 
    map_.info.origin.position.x = -static_cast<double>(width) / 2 * resolution; //0号栅格（高度和宽度方向上的最小栅格）左下角坐标（真实值）
    map_.info.origin.position.y = -static_cast<double>(height) / 2 * resolution;

	/*地图参数*/
	mapinfo.map_position_x=	map_.info.origin.position.x;
	mapinfo.map_position_y=	map_.info.origin.position.y;
	mapinfo.map_width = map_.info.width;           
    mapinfo.map_height= map_.info.height;         
    mapinfo.map_resolution= map_.info.resolution;
 
    frontMapData = new float[width * height];  //水平激光判断每个栅格是否是障碍的置信度
}

MapBuilder::~MapBuilder(){  delete frontMapData;  }
/*
* 入口函数
*/

/*Function 6.1.2.2
* 函数功能 判断激光线在栅格地图内是否经过障碍
*/
bool MapBuilder::getRayCastToObstacle( double angle, double angle_resolution, double range, std::vector<size_t> &raycast)//通过激光点投射返回像素列表
{
    if (range < 1e-10)//消除（0,0,0）误差点
    {
        raycast.clear();
        return false;
    }
    ::map_ray_caster::MapRayCaster mrc;
    //const std::vector<size_t> &ray_to_map_border = mrc.getRayCastToMapBorder(angle,map_.info.height, map_.info.width, 1.1 * angle_resolution);//返回像素的索引列表（当前点到地图边界）， angle_resolution_：180/720----0.25
	const std::vector<size_t> &ray_to_map_border = mrc.getRayCastToMapBorder(angle,map_.info.height, map_.info.width, 0);
	
    const int32_t pixel_range = lround(range * std::max(fabs(cos(angle)), fabs(sin(angle))) / map_.info.resolution);//angle是弧度，pixel_range到达障碍点的像素个数
    int32_t raycast_size;//栅格地图内，当前激光线方向上，无障碍栅格的数量
    bool obstacle_in_map = pixel_range < ray_to_map_border.size();//ray_to_map_border用来存储到达地图边界的像素点

    if (obstacle_in_map)//地图中有障碍
    {
        raycast_size = pixel_range;//pixel像素
    }
    else//地图中没有障碍
    {
        raycast_size = ray_to_map_border.size();
    }

    raycast.clear();
    raycast.reserve(raycast_size);

    for (int32_t i = 0; i < raycast_size; ++i)
    {
        raycast.push_back(ray_to_map_border[i]);//激光到边界或障碍物这一路上的所有点，即可行点
    }

    return obstacle_in_map;//返回指定区域内是否有障碍
}

/*Function 6.1
* 函数功能 栅格地图移动，栅格地图更新
*/
void MapBuilder::updateMap(sensor_msgs::LaserScan scan_in)
//void MapBuilder::updateMap(const sensor_msgs::LaserScanConstPtr &scan_in)
{
	//////////////////////初始化/////////////////////////
	map_.data.assign( map_.info.width *  map_.info.height, labels[0]);
	
	double angle_min = scan_in.angle_min;//第一条扫描线的夹角
	double angle_increment = scan_in.angle_increment;//相邻扫描线的增值（常值）
	//std::cout<<"angle_min: "<<angle_min<<std::endl;
	size_t length = map_.info.width * map_.info.height; //栅格地图栅格个数

	//std::cout<<"length: "<<length<<std::endl;
    //std::cout<<"ranges.size(): "<<scan_in.ranges.size()<<std::endl;

	memset(frontMapData, 0, length*sizeof(float));      //将frontMapData初始化
    /*for(size_t i = 0; i < scan_in.ranges.size(); ++i){
        std::cout<<"ranges["<<i<<"] = "<<scan_in.ranges[i]<<std::endl;
    }*/
	//std::cout<<"scan_in.ranges[0] = "<<scan_in.ranges[0]<<std::endl;
	//std::cout<<"ranges[0]: "<<scan_in->ranges[0]<<std::endl;
	//Step1 水平激光数据判断障碍  //Step3
    //return;
	for(size_t i = 0; i < scan_in.ranges.size(); ++i)   //对每条水平激光线的操作
	{
		//std::cout<<"scan_in.ranges["<<i<<"] = "<<scan_in.ranges[i]<<std::endl;
		double _angle, _range;
		const double angle = angles::normalize_angle(angle_min + i * angle_increment + heading);//heading偏航角（弧度）；angle：当前这条激光线的弧度
		double angle_pre = angle_min + i * angle_increment;
		double angle_trianle;
		_angle = angle;
		_range = scan_in.ranges[i];

		std::vector<size_t> pts;                        //用于存储当前激光点经过的栅格
		const bool obstacle_in_map = getRayCastToObstacle(_angle, angle_increment, _range, pts);//Function 6.1.2.2
		//对每条激光线的操作：map_：栅格地图对象 ，angle：scan.ranges[i]：每条激光线的数据 ，pts：
        if (pts.empty())//没有元素，一般为无效点（0,0,0）返回真
       	{
			continue;
       	}
        if (obstacle_in_map && pts.size() > 2)//检测到障碍
        {
			const size_t last_pt = pts.back();//last_pt为障碍点栅格
			//updatePointOccupancy(true, last_pt, frontMapData, length);//true，代表更新的是障碍栅格
			frontMapData[last_pt] = 1.0;
			pts.pop_back();//清除尾部数据
		}
		//updatePointsOccupancy(false, pts, frontMapData, length);
		for(int k=0; k<pts.size(); k++) frontMapData[pts[k]] = 0.1;
	}
	
		//Step2 整理剩余标签  //Step6
    for(int i=0; i<length; ++i)     //遍历栅格地图的所有栅格
    {
		
       	if (frontMapData[i] < 0.5 && frontMapData[i] > 0.0)   //小于阈值，水平激光认为是可行点
        {
			map_.data[i] = labels[0];
       	}
        else if (frontMapData[i] > 0.5)   //大于阈值，水平激光认为是障碍点
        {
			map_.data[i] = labels[1];
			//mapData[i].isH = 1;

			int a_x=0;int a_y=0;//坐标
			int l=map_.info.width;

			a_x=(int)(map_.info.width/2)-(int)(i/map_.info.width);
			a_y=(int)(map_.info.width/2)-(int)(i%map_.info.width);
			if(abs(abs(a_x)-l/2)<5||abs(abs(a_y)-l/2)<5)continue ;//虑除点
        }

 		if (frontMapData[i] > 0.5)   //大于阈值，水平激光认为是障碍点
		{

		}
	}

}

void MapBuilder::setPoseAndHeading(double x, double y, double z, double angle)//更新机器人位置
{
    mtx_pose.lock();
    now_x = x;
    now_x = y;
    now_x = z;
    heading = angle;
    //std::cout<<" ----> "<<"heading: "<<heading<<std::endl;
    map_.info.origin.position.x = x-static_cast<double>(map_.info.width) / 2 * map_.info.resolution; //更新地图在Rviz中的位置
    map_.info.origin.position.y = y-static_cast<double>(map_.info.height) / 2 * map_.info.resolution;
    mtx_pose.unlock();

}


nav_msgs::OccupancyGrid MapBuilder::getMap()
{
	boost::mutex::scoped_lock lock(mtx_map);
	
    return map_;
}

}//end of namespace
/************************/


