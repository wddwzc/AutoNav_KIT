#ifndef LOCAL_MAP_PARAMETER_H
#define LOCAL_MAP_PARAMETER_H

#include <vector>
#include <map>
using namespace std;

const int MAP_WIDTH=150;
const int MAP_HEIGHT=150;


/*地图信息*/
struct Map_info
{
	/*栅格地图左下角栅格的全局坐标*/
	float map_position_x;     
	float map_position_y;

	int map_height;	//地图高
	int map_width;	//地图宽
	float map_resolution;//地图分辨率
};


//地图相关参数
const unsigned char labels[4] = {1,120,180,240};       	//栅格标签 0可行，1障碍，2动态障碍，3道牙子
//const unsigned char labels[4] = {1,80,100,130};
//障碍参数
const float threshold_height=0.25;			//可通行高度（栅格内点的高度差）
const float mini_speed=0.5;					//速度大于0.5m/s的障碍认为是动态障碍
const int pic_size=720;						//可视化界面大小

#endif  // LOCAL_MAP_PARAMETER_H
