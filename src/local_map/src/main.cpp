#include <ros/ros.h>
#include <iostream>
#include <ros/console.h>
#include <local_map/map_builder.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Quaternion.h>

/*消息订阅*/
ros::Subscriber* g_pPointcloudSubscriber;	//订阅点云的原始数据
ros::Subscriber* g_pPoseSubscriber;		//订阅激光扫描仪的位姿[x,y,z,yaw]

/*消息发布*/
ros::Publisher* g_pMap_publisher;			//发布地图消息

std::string strNodeName = "local_map";			//节点名
std::string strPubMapTopicName ="local_map";	//发布消息名
//std::string strPointcloudTopicName = "/rslidar_points";//接收激光传感器原始消息
// std::string strPointcloudTopicName = "/hokuyoScan";
std::string strPointcloudTopicName = "/scan";
std::string strPoseTopicName = "/chatter";//接收机器人位姿
//std::string strPoseTopicName = "/air_pose";//接收机器人位姿
local_map::MapBuilder* map_builder_ptr;

bool update = false;

/*激光消息回调函数*/
void PointcloudCallback(const sensor_msgs::LaserScanConstPtr &msg)
{
	//map_builder_ptr->setPoseAndHeading(0, 0, 0, 0);
	//if(update)
	if(1)
	{
		/*for(size_t i = 0; i < msg->ranges.size(); ++i){
			std::cout<<"ranges["<<i<<"] = "<<msg->ranges[i]<<std::endl;
		}*/
		/*每接收到一次激光数据时更新一次地图*/
		map_builder_ptr->updateMap(*msg);
		/*发布更新后的新地图，地图更新频率与激光消息频率一致*/
		g_pMap_publisher->publish(map_builder_ptr->getMap());
		/*标志位，防止激光消息和位姿消息频率差距过大*/
		update = false;

	}

}

/*位姿消息回调函数*/

void PoseCallback(nav_msgs::Odometry pose)
{

	double x = 0,y = 0,z = 0,w = 0;				//姿态四元数
	double roll, pitch, yaw; 	//姿态欧拉角：单位弧度

	x = pose.pose.pose.orientation.x;  //注意，这里带着转换
	y = pose.pose.pose.orientation.y;
	z = pose.pose.pose.orientation.z;
	w = pose.pose.pose.orientation.w;

  	tf::Matrix3x3(tf::Quaternion(x, y, z, w)).getRPY(roll, pitch, yaw);
	//std::cout<<"yaw: "<<yaw<<" ----> "<<yaw*180/3.1415926<<std::endl;
    map_builder_ptr->setPoseAndHeading(pose.pose.pose.position.x, pose.pose.pose.position.y,pose.pose.pose.position.z, yaw);
    update = true;
}


int main(int argc, char** argv)
{

    ros::init(argc, argv, strNodeName);
    ros::NodeHandle node;

	/*地图初始化*/
	unsigned short map_width  = MAP_WIDTH;//地图宽
	unsigned short map_height = MAP_HEIGHT;//地图高
	double map_resolution = 0.1;	//地图分辨率(m)
	map_builder_ptr = new local_map::MapBuilder(map_width, map_height, map_resolution);

	/*订阅位姿消息*/
	g_pPoseSubscriber = new ros::Subscriber;
	*g_pPoseSubscriber = node.subscribe(strPoseTopicName, 10000, &PoseCallback);

	/*订阅激光消息*/
 	g_pPointcloudSubscriber = new ros::Subscriber;
    *g_pPointcloudSubscriber = node.subscribe(strPointcloudTopicName, 10000, PointcloudCallback);
    //*g_pPointcloudSubscriber = node.subscribe<sensor_msgs::LaserScan>(strPointcloudTopicName, 100, &PointcloudCallback);
 	/*发布地图消息*/   
 	g_pMap_publisher = new ros::Publisher;
	*g_pMap_publisher = node.advertise<nav_msgs::OccupancyGrid>(strPubMapTopicName, 2000);

    ros::spin();

	delete map_builder_ptr;

}




