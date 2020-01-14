#include "Base_control/ControlCmd.h"
#include "ros/ros.h"
#include <ros/time.h>
#include <thread>
#include <mutex>
#include "Insrob_server/Speed.h"

#include "Insrob_server/global.h"
#include "Insrob_server/orient.h"
#include "Insrob_server/vsocket.h"
#include "Insrob_server/vpoint.h"

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

VSocket *thread_socket = NULL;
bool is_send_pc = false;
bool is_send_pos = false;
bool is_connect_on = false;
bool is_send_data = false;
bool thread_run = false;
char *pbufferPC = NULL;
char bufferPos[400];
int socket_cmd_ret = 0;
pcl::PointCloud<pcl::PointXYZI>::Ptr airpoints(new pcl::PointCloud<pcl::PointXYZI>());
float current_x = 0.0, current_y=0.0, current_z=0.0;

bool is_stop = true;//标志是否暂停
int lastlinear = 0;
int lasttwist = 0;
int run_status = 0;

ros::ServiceClient client;

void speed_change(CmdType cmd){
	int lastlinear_add = 0;
	int lasttwist_add = 0;
	Base_control::ControlCmd speed_srv;
	switch (cmd) {
		case CMD_MOVE_FORWARD: {
			lastlinear_add = 1;
			break;
		}
		case CMD_MOVE_BACK: {
			lastlinear_add = -1;
			break;
		}
		case CMD_MOVE_LEFT: {
			lasttwist_add = 1;
			break;
		}
		case CMD_MOVE_RIGHT: {
			lasttwist_add = -1;
			break;
		}
		case CMD_MOVE_STOP: {
			lastlinear = 0;
			lasttwist = 0;
			lastlinear_add = 0;
			lasttwist_add = 0;
		}
		case CMD_MOVE_KEEP: {
			lastlinear_add = 0;
			lasttwist_add = 0;
		}
		default: {
			break;
		}
	}
	if ((lastlinear + lastlinear_add) <= 4 && (lastlinear + lastlinear_add) >= -4) {
		lastlinear += lastlinear_add;
	}
	if ((lasttwist + lasttwist_add) <= 4 && (lasttwist + lasttwist_add) >= -4) {
		lasttwist += lasttwist_add;
	}
	speed_srv.request.xx = 100 * lastlinear;
	speed_srv.request.yy = 100 * lasttwist;
	if (client.call(speed_srv)) {
		ROS_INFO("Set vel=%d,omiga=%d,res=%f ",speed_srv.request.xx, speed_srv.request.yy, speed_srv.response.zz);
	}
	else {
		ROS_ERROR("Failed to call service add_two_ints");
	}
}

void AnalyzeData (CmdType cmd, unsigned int len) {
	switch (cmd) {
		case CMD_DATA_GETCLOUD: {
			is_send_pc = true;
			cout << "transfer point cloud" << endl;
			break;
		}
		case CMD_DATA_ENDCLOUD: {
			is_send_pc = false;
			cout << "stop transfering cloud" << endl;	
			break;
		}
		case CMD_DATA_GETPOSE: {
			is_send_pos = true;
			cout << "transfer robot pose" << endl;
			break;
		}
		case CMD_DATA_ENDPOSE: {
			is_send_pos = false;
			cout << "stop transfering pose" << endl;	
			break;
		}
		default: {
			break;
		}
	}
}

void AnalyzeInstruction(CmdType cmd) {
	switch (cmd) {
		case CMD_INSTRUCTION_INTERRUPT: {
			cout << "interrupt!!!" << endl;
			is_send_data = false;
			thread_socket->close_current_sock();
			is_connect_on = false;
			CPackage pack;
			pack.msgType = CMD_TYPE_INSTRUCTION;
			pack.msgData = CMD_INSTRUCTION_INTERRUPT;
			thread_socket->SendPC((char*)(&pack), sizeof(CPackage));
			break;
		}
		default: {
			break;
		}
	}
}

void point_repeat(const sensor_msgs::PointCloud2::ConstPtr& pointsData) {
	int len = 0;
	unsigned int cnt = 0;
	//将ROS点云消息 转换为PCL点云格式
	airpoints->clear();
	pcl::fromROSMsg(*pointsData, *airpoints);
	//整理数据包头
	CPackage pack;
	pack.msgType = CMD_TYPE_DATA;
	pack.msgData = CMD_DATA_GETCLOUD;
	pack.msgLen = airpoints->points.size();
	char *str = pbufferPC;
	memcpy(str, (char*)(&pack), sizeof(CPackage));
	str += sizeof(CPackage);
	len += sizeof(CPackage);
	//整理数据包内容
	VPointI point;
	for (size_t i = 0; i < airpoints->points.size(); ++i) {
		point.x = airpoints->points[i].z;  //注意，这里带着坐标轴转换
		point.y = airpoints->points[i].x;
		point.z = airpoints->points[i].y;
		point.intensity = airpoints->points[i].intensity; 
		memcpy(str, (char*)(&point), sizeof(VPointI));
		len += sizeof(VPointI);
		str += sizeof(VPointI);
		cnt++;
		//ROS_INFO("data_length: %d point_number: %d", len, cnt);
	}
	//ROS_INFO("Already to transfered [%d]!!!", pack.msgLen);
	//if(!is_send_pc) return;
	if (thread_socket->SendPC(pbufferPC, len)) {
		ROS_INFO("Already to transfered [%d] [%d]!!!", cnt, len);
		//is_send_data = false;
	}
}

void Map_Callback(const sensor_msgs::PointCloud2::ConstPtr& pointsData) {
	if(!is_send_data || !is_send_pc) {
		//ROS_INFO("no transfering map");
		return;
	}
	point_repeat(pointsData);
}

void Pose_Callback(const nav_msgs::Odometry::ConstPtr& poseData) {
	if (!is_send_data || !is_send_pos) {
		//ROS_INFO("no transfering pose");
		return;
	}
	float px,py,pz;			//位置
	double x,y,z,w;				//姿态四元数
	double yaw, pitch, roll; 	//姿态欧拉角：单位弧度
	px = poseData->pose.pose.position.z;
	py = poseData->pose.pose.position.x;
	pz = poseData->pose.pose.position.y;
	x = poseData->pose.pose.orientation.z;  //注意，这里带着转换
	y = poseData->pose.pose.orientation.x;
	z = poseData->pose.pose.orientation.y;
	w = poseData->pose.pose.orientation.w;
	tf::Matrix3x3(tf::Quaternion(x, y, z, w)).getRPY(roll, pitch, yaw);
	//暂存一下当前位置，过滤激光点使用
	current_x = px;
	current_y = py;
	current_z = pz;
//	ROS_INFO("the x is %f,the y is %f,the z is %f",px,py,pz);
//	ROS_INFO("the roll is %f,the pitch is %f,the yaw is%f",roll,pitch,yaw);
	ros::Time stamp = poseData->header.stamp;
	double sec = poseData->header.stamp.toSec();

	CPackage pack_pose;
	pack_pose.msgType = CMD_TYPE_DATA;
	pack_pose.msgData = CMD_DATA_GETPOSE;
	pack_pose.msgLen = sizeof(VPoseStamped);
	//整理数据包内容
	int len = 0;
	char *str = bufferPos;
	memcpy(str,(char*)(&pack_pose),sizeof(CPackage));
	len += sizeof(CPackage);
	str += sizeof(CPackage);

	VPoseStamped the_pose;
	the_pose.timestamp = sec;
    //pose_num++;
    //the_pose.pose_id = 0;
	the_pose.x = px;
	the_pose.y = py;
	the_pose.z = pz;
	the_pose.i = x;
	the_pose.j = y;
	the_pose.k = z;
	the_pose.w = w;
//	the_pose.yaw = yaw * 180.0 / 3.1415926;
//	the_pose.pitch = pitch * 180.0 / 3.1415926;
//	the_pose.roll = roll * 180.0 / 3.1415926;
    the_pose.yaw = yaw;
    the_pose.pitch = pitch;
    the_pose.roll = roll;

	memcpy(str,(char*)(&the_pose),sizeof(VPoseStamped));
	len += sizeof(VPoseStamped);
	str += sizeof(VPoseStamped);

	if (thread_socket->SendPC(bufferPos, len)) {
		ROS_INFO("Already transfered pose!!!");
	}
	memset(bufferPos,0,400);
} 

void AnalyzePackage(CPackage pack) {
	CmdType type = pack.msgType;
	CmdType data = pack.msgData;
	unsigned int datalen = pack.msgLen;
	switch (type) {
        case CMD_TYPE_DATA: {
            cout << "CMD_TYPE_DATA" << endl;
            AnalyzeData(data, datalen);
            break;
        }
        case CMD_TYPE_MOVEMENT: {
            cout << "CMD_TYPE_MOVEMENT" << endl;
            speed_change(data);
            break;
        }
        case CMD_TYPE_INSTRUCTION: {
            cout << "CMD_TYPE_INSTRUCTION" << endl;
            AnalyzeInstruction(data);
            break;
        }
        case CMD_NONE: {
        	break;
        }
        case CMD_OFF: {
			cout << "off!!!" << endl;
			is_send_data = false;
			thread_socket->close_current_sock();
			is_connect_on = false;
			break;
        }
    }
}

void interation_thread()
{
	std::lock_guard<std::mutex> guard(mutex);
	ros::Rate loop(30);
	while (thread_run) {
		if(!is_connect_on) {
			if (thread_socket->Accept()) {
				is_connect_on = true;
				is_send_data = true;
				std::cout << "Connected!" << std::endl;
			}
		}
		if(is_connect_on) {
			CPackage ret = thread_socket->RecvData();
			AnalyzePackage(ret);
		}
		loop.sleep();
	}
}

int main(int argc, char **argv){
	ros::init(argc, argv, "app_Insrob_server");
	ros::NodeHandle n;
	ros::Subscriber map_sub = n.subscribe("/air_points", 2, Map_Callback);
	ros::Subscriber pose_sub = n.subscribe("/air_pose", 2, Pose_Callback);
	client = n.serviceClient<Base_control::ControlCmd>("Controler");

	if (thread_socket == NULL)
		thread_socket = new VSocket;
	pbufferPC = new char[2000000];
	ROS_INFO("server was initialized");

	thread_run = true;
	thread thread_commu(interation_thread);

    ros::Rate loop_rate(15);
	while(ros::ok()) {
		// std::cout << "waiting for linking" << std::endl;

		ros::spinOnce();
		loop_rate.sleep();
	}
	thread_run = false;
	thread_commu.join();
	delete []pbufferPC;
	cout << "over" << endl;
	return 0;
}
