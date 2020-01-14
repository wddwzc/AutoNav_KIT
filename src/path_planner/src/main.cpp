#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <ros/console.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include "visualization_msgs/Marker.h"
#include "Base_control/ControlCmd.h"
#include "path_planner/point.h"
#include "path_planner/Pose.h"
#include "tf/transform_datatypes.h"
#include "path_planner/computetool.h"
#include "path_planner/Clothoid_arcfinder.h"
#include "path_planner/global_path.h"
#include "path_planner/Bspline.h"	
#include "path_planner/smooth.h"
#include "path_planner/Bspline_Mtrajfinder.h"


ros::Publisher Key_point;
ros::Publisher Node_text;
ros::Publisher smoothpath;
ros::Publisher orpath;
ros::Publisher Conpointpuber;

// //for test
ros::Publisher centerbiaozhun;
ros::Publisher MakerPosepub;

//publishers and subscribers
ros::Subscriber* g_pMapSubscriber;
ros::Subscriber* g_pPoseSubscriber;

//ros ServiceClient cmd to robot
ros::ServiceClient* g_pclienter;
bool autorun = true;
float linearratio = 1.0; 



//special names
std::string strNodeName = "path_planner";


std::string strMapTopicName = "/local_map";
std::string strPoseTopicName = "/chatter";

std::string strCmdServiceName = "Controler";

Insrobpose Robpose;
CPoint2d subgoal;
bool isPoseOk = false;
bool isReachgoal = false;

///////////////////////Global planner////////////
global_path global_path_;


/////////////////////////////Clothoid_arcfinde class////////////////
Clothoid_arcfinder Clothoid_arcfinderplanner_(global_path_.key_point);
bool flagClothoid_arcfinder = false;
int pathnum = 0;
std::vector<geometry_msgs::Point32> leftpolygon;
std::vector<geometry_msgs::Point32> rightpolygon;
ros::Publisher pubPolgon;
ros::Publisher pubNodes;

////////////////////////////////Bspline_Mtrajfinder////////////////////
Bspline_Mtrajfinder Bspline_Mtrajfinder_(global_path_.key_point);
bool flagBspline_Mtrajfinder = true;
std::vector<CPoint2d> Sugoalps;
ros::Publisher traj_pub;					//rviz上显示出最优轨迹的消息
ros::Publisher Mutraj_pub[200];				//用于发布轨迹消息，提供给rviz上显示调试使用




///////////////////////////pub goal and rob pose/////////////////////
ros::Publisher pubGoaler;
ros::Publisher pubRobposer;




// std::ifstream infile;

void HandleMapCallback(nav_msgs::OccupancyGrid msg)
{
	std::cout<<"************ GET OccupancyGrid MAP.......................... "<<std::endl;
	std::cout<<msg.header.frame_id<<std::endl;
	std::cout<<msg.info.width<<" "<<msg.info.height<<" "<<msg.info.resolution<<std::endl;
	std::cout<<msg.info.origin.position.x<<" "<<msg.info.origin.position.y<<" "<<msg.info.origin.position.z<<std::endl;
	std::cout<<Robpose.yaw<<" "<<Robpose.roll<<" "<<Robpose.pitch<<std::endl;
		
	//局部坐标系下机器人rviz显示位置函数
	// pubRobpose(msg.header.frame_id,0.0,0.0,
						// Robpose.z,Robpose.rx,
						// Robpose.ry,Robpose.rz,Robpose.rw,0.0);
						
	//世界坐标系下机器人rviz显示位置函数
	// pubRobpose(msg.header.frame_id,Robpose.x,Robpose.y,
						// Robpose.z,Robpose.rx,
						// Robpose.ry,Robpose.rz,Robpose.rw,Robpose.yaw);

	if(isPoseOk && flagClothoid_arcfinder && !isReachgoal)
	{
		std::cout<<"Goal is ...... "<<std::endl;
		std::cout<<Clothoid_arcfinderplanner_.Goalpoint.x<<" "<<Clothoid_arcfinderplanner_.Goalpoint.y<<" "<<Clothoid_arcfinderplanner_.Goalpoint.pointangle<<std::endl;

		CPoint2d Rob;
		Rob.x = Robpose.x;
		Rob.y = Robpose.y;
		Rob.pointangle = Robpose.yaw;


		if(Clothoid_arcfinderplanner_.ChechReachG(Rob))
		{
			std::cout<<"reach final goal"<<std::endl;
			isReachgoal = true;

			Base_control::ControlCmd srv;
			srv.request.xx = 0;
			srv.request.yy = 0;
			if(autorun)
			{
				g_pclienter->call(srv);		
			}

			std::cout<<"output .... "<<srv.request.xx<<" "
					 <<srv.request.yy<<std::endl;

			return;
		}
		else
		{
			
			Clothoid_arcfinderplanner_.Getsubgoal(msg.header.frame_id,Rob,subgoal);

			Clothoid_arcfinderplanner_.updateMap(msg);
		
			float plannerrobx = Robpose.x;
			float plannerroby = Robpose.y;
			float plannerrobth = Robpose.yaw;


			std::cout<<"pubgoal and robpose"<<std::endl;

			pubGoal(msg.header.frame_id,subgoal.x,subgoal.y,subgoal.pointangle);
			
			Clothoid_arcfinderplanner_.SetS_Gpoint(0.0,
												   plannerrobx,plannerroby,
												   Robpose.yaw,subgoal.x,
												   subgoal.y,
												   subgoal.pointangle);
			float linear = 0.0;
			float twist = 0.0;
			
			if(Clothoid_arcfinderplanner_.make_plan())
			{
					pathnum = 0;
					pubfianalpath(msg.header.frame_id,Clothoid_arcfinderplanner_.final_path);

					// int indexnode = Clothoid_arcfinderplanner_.final_path.size();

					//control cmd
					float Clothoid_Dcruv = 0.0;

					std::cout<<"Clothoid_arcfinderplanner_.final_path.size() "<<Clothoid_arcfinderplanner_.final_path.size()<<std::endl;

					int indexsize = Clothoid_arcfinderplanner_.final_path.size()-2;


					if(Clothoid_arcfinderplanner_.final_path.size()>2)
					{
						
						std::cout<<"indexsize "<<indexsize<<std::endl;
						//indexsize = indexsize - 1;
						std::cout<<"indexsize------ "<<indexsize<<std::endl;
						Clothoid_Dcruv = Clothoid_arcfinderplanner_.final_path[indexsize].Cruvrate;
					}
					else
					{
						Clothoid_Dcruv = Clothoid_arcfinderplanner_.
										 final_path[Clothoid_arcfinderplanner_.final_path.size()-1].Cruvrate;
					}

					float sumCruvrate = 0.0; 

					
					sumCruvrate = Clothoid_arcfinderplanner_.final_path[indexsize].Cruvrate 
								  + Clothoid_arcfinderplanner_.final_path[indexsize + 1].Cruvrate;	

					

					std::cout<<"sumCruvrate "<<sumCruvrate<<std::endl;

					std::cout<<"Clothoid_Dcruv "<<Clothoid_Dcruv<<std::endl;
					
					//int choose = 10.0 * Clothoid_Dcruv;

					// std::cout<<"you get is ...."<<choose<<std::endl;


					float finalCruv = Clothoid_arcfinderplanner_.final_path[indexsize].Cruv;
					//float startCruv = Clothoid_arcfinderplanner_.final_path[indexsize + 1];

					pubClothoid_DcruvVelangTwist(finalCruv,Clothoid_Dcruv,linear,twist);

					
					Clothoid_arcfinderplanner_.CarCruv = Clothoid_Dcruv * linear 
															* 0.1 + Clothoid_arcfinderplanner_.CarCruv;
					
					if(Clothoid_arcfinderplanner_.CarCruv>Clothoid_arcfinderplanner_.Max_Cruv)
					{
						Clothoid_arcfinderplanner_.CarCruv = 
															Clothoid_arcfinderplanner_.Max_Cruv;
					}
					
					
					if(Clothoid_arcfinderplanner_.CarCruv<Clothoid_arcfinderplanner_.Min_Cruv)
					{
						Clothoid_arcfinderplanner_.CarCruv = 
															Clothoid_arcfinderplanner_.Min_Cruv;
					}
					std::cout<<"pub vel "<<linear<<" "<<twist<<" "<<Clothoid_arcfinderplanner_.CarCruv<<std::endl;
			}
			else
			{

						std::cout<<"failed "<<std::endl;
						std::cout<<"path_planner.final_path.size() "<<Clothoid_arcfinderplanner_.final_path.size()<<std::endl;
						pathnum = pathnum + 1;
						
						linear = 0.0;
						twist = 0.0;
						
						std::cout<<"pub vel "<<linear<<" "<<twist<<std::endl;

						Base_control::ControlCmd srv;
						srv.request.xx = 0;
						srv.request.yy = 0;

						if(autorun)
						{
							g_pclienter->call(srv);		
						}

						std::cout<<"output .... "<<srv.request.xx<<" "
								 <<srv.request.yy<<std::endl;


						// if(pathnum>10)
						// {
						// 	local_map::clearOldData srv1;
						// 	srv1.request.isClearAll = false;
						// 	m_client_localmap_clear_olddata.call(srv1);
						// 	wait_for_a_moment_in_g0.00348372 -0.000343896 -0.00153789rid_callback(1.5);
						// 	pathnum = 0;
						// }

			}

		}
	}
	
	if(isPoseOk && flagBspline_Mtrajfinder && !isReachgoal)
	{
		std::cout<<"get into Bspline_Mtrajfinder......"<<std::endl;
		
		// 把局部地图转换成算法中使用的数据格式
		Bspline_Mtrajfinder_.Setmapindex(msg);
		
		if(Bspline_Mtrajfinder_.Rechgoal())
		{
			std::cout<<"reach final goal"<<std::endl;
			isReachgoal = true;

			Base_control::ControlCmd srv;
			srv.request.xx = 0;
			srv.request.yy = 0;
			if(autorun)
			{
				g_pclienter->call(srv);		
			}
			std::cout<<"output .... "<<srv.request.xx<<" "
					 <<srv.request.yy<<std::endl;

			return;
		}
		else
		{
			Bspline_Mtrajfinder_.Rob.x = Robpose.x;
			Bspline_Mtrajfinder_.Rob.y = Robpose.y;
			Bspline_Mtrajfinder_.Rob.pointangle = Robpose.yaw;
			
			// Bspline_Mtrajfinder_.Setmapindex(msg);
			
			Bspline_Mtrajfinder_.Getsubgoal(msg.header.frame_id,subgoal);
			
			// std::cout<<"index search "<<Bspline_Mtrajfinder_.refer_pathindex<<std::endl;

			std::cout<<"subgoal "<<subgoal.x<<" "<<subgoal.y
					    <<" "<<subgoal.pointangle<<std::endl;
						
			pubGoal(msg.header.frame_id,subgoal.x,subgoal.y,subgoal.pointangle);

			pubRobpose(msg.header.frame_id,Robpose.x,Robpose.y,
						Robpose.z,Robpose.rx,
						Robpose.ry,Robpose.rz,Robpose.rw,Robpose.yaw);
						
			Bspline_Mtrajfinder_.Generatraj(msg.header.frame_id);
			
			if(Bspline_Mtrajfinder_.pathID == -1)
			{
				Base_control::ControlCmd srv;
				srv.request.xx = 0;
				srv.request.yy = 0;
				if(autorun)
				{
					g_pclienter->call(srv);
				}
				std::cout<<"output .... "<<srv.request.xx<<" "
					 <<srv.request.yy<<std::endl;

				std::cout<<"no path"<<std::endl;
			}
			else
			{
				//确定轨迹的ID号，将选择轨迹传至Sugoalps轨迹向量中，以便获取控制点使用
				if(Bspline_Mtrajfinder_.pathID>=Bspline_Mtrajfinder_.CLnum)
				{

					int num = Bspline_Mtrajfinder_.pathID
							  - Bspline_Mtrajfinder_.CLnum;
							  
					std::cout<<"updating____you choose left traj,choose path ID "<<num<<std::endl;

					Sugoalps = Bspline_Mtrajfinder_.LG_Paths[num];

					// 修改了此处distoBob的值
					CPoint2d Subgoal = Bspline_Mtrajfinder_.getgoalpoint(msg.header.frame_id,Sugoalps);

					pubconpoint(msg.header.frame_id,Subgoal);
					
					///control cmd .......
					Cmd_level CmdtoRob;
						
					Bspline_Mtrajfinder_.ComputeCmd(msg.header.frame_id,Subgoal,CmdtoRob);
						
					std::cout<<"CmdtoRob "<<CmdtoRob.linear<<" "<<CmdtoRob.twist<<std::endl;

					Base_control::ControlCmd srv;
					srv.request.xx = CmdtoRob.linear * 1000;
					srv.request.yy = CmdtoRob.twist * 1000;

					if(autorun)
					{
						g_pclienter->call(srv);		
					}

					std::cout<<"output .... "<<srv.request.xx<<" "
					         <<srv.request.yy<<std::endl;					
						
				}
				else
				{
					std::cout<<"you choose right traj,choose path ID "<<Bspline_Mtrajfinder_.pathID<<std::endl;
					Sugoalps = Bspline_Mtrajfinder_.RG_Paths[Bspline_Mtrajfinder_.pathID];
					

					CPoint2d Subgoal = Bspline_Mtrajfinder_.getgoalpoint(msg.header.frame_id,Sugoalps);

					pubconpoint(msg.header.frame_id,Subgoal);
					
					///control cmd .......
					Cmd_level CmdtoRob;
						
					Bspline_Mtrajfinder_.ComputeCmd(msg.header.frame_id,Subgoal,CmdtoRob);
						
					std::cout<<"CmdtoRob "<<CmdtoRob.linear<<" "<<CmdtoRob.twist<<std::endl;

					Base_control::ControlCmd srv;
					srv.request.xx = CmdtoRob.linear * 1000;
					srv.request.yy = CmdtoRob.twist * 1000;

					if(autorun)
					{
						g_pclienter->call(srv);		
					}

					std::cout<<"output .... "<<srv.request.xx<<" "
					         <<srv.request.yy<<std::endl;
				}
				
				pubtrajs(msg.header.frame_id);
			}
		}
		
		
	}


	//for test
	nav_msgs::Path temnowpath;
	temnowpath.header.stamp=ros::Time::now();
	temnowpath.header.frame_id="/camera_init";
	for(int j=0;j<Bspline_Mtrajfinder_.partreferpath.size();j++)
	{
		geometry_msgs::PoseStamped tempose;
		tempose.header.stamp=temnowpath.header.stamp;
		tempose.header.frame_id=temnowpath.header.frame_id;
		tempose.pose.position.x=Bspline_Mtrajfinder_.partreferpath[j].x;
		tempose.pose.position.y=Bspline_Mtrajfinder_.partreferpath[j].y;
		tempose.pose.position.z=0;
		temnowpath.poses.push_back(tempose);
	}

	centerbiaozhun.publish(temnowpath);

	/*visualization_msgs::Marker marker1;
	marker1.header.frame_id = "/camera_init";
	marker1.header.stamp = ros::Time::now();
	marker1.ns = "/show_robot";

	marker1.action = visualization_msgs::Marker::ADD;
	marker1.type = visualization_msgs::Marker::MESH_RESOURCE;
	marker1.mesh_resource = "package://path_planner/meshes/R2-D2/R2-D2.dae";


	marker1.scale.x = 1;
	marker1.scale.y = 1;
	marker1.scale.z = 1;
	marker1.lifetime = ros::Duration();
	marker1.id =5;
	marker1.pose.position.x=Robpose.x;
	marker1.pose.position.y=Robpose.y;
	marker1.pose.position.z=0;
	marker1.pose.orientation.x = 0.0;
	marker1.pose.orientation.y = 0.0;
	marker1.pose.orientation.z = sin((Robpose.yaw+1.5707)/2);
	marker1.pose.orientation.w = cos((Robpose.yaw+1.5707)/2);
	marker1.color.a=1.0;
	marker1.color.r = 1.0;
	marker1.color.g = 1.0;
	marker1.color.b = 1.0;		
	MakerPosepub.publish(marker1);*/
}

void HandlePoseCallback(nav_msgs::Odometry poseData)
{
	std::cout<<"############ GET NEW ROB POSE ...................... "<<std::endl;
	isPoseOk = true;

	double yaw, pitch, roll; 
	tf::Matrix3x3(tf::Quaternion(poseData.pose.pose.orientation.x, 
								 poseData.pose.pose.orientation.y, 
								 poseData.pose.pose.orientation.z, 
								 poseData.pose.pose.orientation.w)).getRPY(roll,pitch,yaw);
	

	
	Robpose.x = poseData.pose.pose.position.x;
	Robpose.y = poseData.pose.pose.position.y;
	Robpose.z = poseData.pose.pose.position.z;
	
	Robpose.yaw = yaw; //pian hang jiao du cha fu hao 
	Robpose.pitch = pitch;
	Robpose.roll = roll;
	Robpose.rx = poseData.pose.pose.orientation.x;
	Robpose.ry = poseData.pose.pose.orientation.y;
	Robpose.rz = poseData.pose.pose.orientation.z;
	Robpose.rw = poseData.pose.pose.orientation.w;

	std::cout<<Robpose.x<<" "<<Robpose.y<<" "<<Robpose.z<<std::endl;
	
	std::cout<<Robpose.yaw<<" "<<Robpose.pitch<<" "<<Robpose.roll<<std::endl;

	std::cout<<Robpose.rx<<" "<<Robpose.ry<<" "<<Robpose.rz<<" "<<Robpose.rw<<std::endl;



	//拓扑地图全局节点显示
	//////////// for show ///////////////
	visualization_msgs::Marker marker1;
	marker1.header.frame_id = "/camera_init";
	marker1.header.stamp = ros::Time::now();
	marker1.type =visualization_msgs::Marker::SPHERE_LIST;
	marker1.action = visualization_msgs::Marker::ADD;
/*	marker1.scale.x = 1;
	marker1.scale.y = 1;
	marker1.scale.z = 1;
*/

	marker1.scale.x = 0.3;
	marker1.scale.y = 0.3;
	marker1.scale.z = 0.3;


/*	marker1.scale.x = 2;
	marker1.scale.y = 2;
	marker1.scale.z = 2;*/

	// marker1.scale.x = 6;
	// marker1.scale.y = 6;
	// marker1.scale.z = 6;

	marker1.lifetime = ros::Duration();
	marker1.id =110;

	for(int o=0;o<global_path_.Nodeset.size();o++)
	{
			geometry_msgs::Point temppoint;
			std_msgs::ColorRGBA color;
			temppoint.x=global_path_.Nodeset[o].x;
			temppoint.y=global_path_.Nodeset[o].y;
			// temppoint.z=1.0;
			temppoint.z=0.0;
		
			marker1.points.push_back(temppoint);
			color.a=1.0;
			color.r=1.0;
			color.g=1.0;
			color.b=1.0;
			marker1.colors.push_back(color); 	
	}

	visualization_msgs::MarkerArray markerArray;
	for(int j=0;j<global_path_.Nodeset.size();j++)
	{
		
	 	visualization_msgs::Marker marker1;
	 	marker1.header.frame_id = "/camera_init";
	 	marker1.header.stamp = ros::Time();
	 	marker1.type =visualization_msgs::Marker::TEXT_VIEW_FACING;
	 	marker1.action = visualization_msgs::Marker::ADD;
		
	 	marker1.scale.x = 1;
	 	marker1.scale.y = 1;
	 	marker1.scale.z =1;

		// marker1.scale.x = 10;
	 // 	marker1.scale.y = 10;
	 // 	marker1.scale.z =10;

	 	/*marker1.scale.x = 2;
	 	marker1.scale.y = 2;
	 	marker1.scale.z = 2;*/

	 	marker1.color.a = 1.0;
	 	marker1.color.r=0.0;
	 	marker1.color.g=0.0;
	 	marker1.color.b=0.0;

	 	std::ostringstream ss;
	 	ss<<(global_path_.Nodeset[j].nodeID);
	 	marker1.id = j+10;
	 	//std::cout<<marker1.id<<std::endl;
	 	marker1.text = ss.str();
	 	//std::cout<<marker1.text<<std::endl;
	 	marker1.pose.position.x = global_path_.Nodeset[j].x;
	 	marker1.pose.position.y = global_path_.Nodeset[j].y;
	 	//marker1.pose.position.z = 4;
		marker1.pose.position.z = 2;


	 // 	if(j>=9 && j<19)
		// {
		// 	marker1.pose.position.x = marker1.pose.position.x + 4;
		// 	marker1.pose.position.y = marker1.pose.position.y + 4;
		// }

		// if(j>=20 && j<=25)
		// {
		// 	marker1.pose.position.x = marker1.pose.position.x - 4;
		// 	marker1.pose.position.y = marker1.pose.position.y - 4;
		// }

		// if(j>=0 || j<9)
		// {
		// 	marker1.pose.position.x = marker1.pose.position.x + 2;
		// 	marker1.pose.position.y = marker1.pose.position.y + 2;
		// }

		// if(j==27 || j==28 || j==19)
		// {
		// 	marker1.pose.position.x = marker1.pose.position.x + 6;
		// 	marker1.pose.position.y = marker1.pose.position.y + 6;
		// }

	 // 	marker1.pose.position.z = 14;
	 	marker1.pose.orientation.x = 0.0;
   		marker1.pose.orientation.y = 0.0;
   		marker1.pose.orientation.z = 0.0;
   		marker1.pose.orientation.w = 1.0;
	 	markerArray.markers.push_back(marker1);
	}

	//原始全局路径和平滑全局路径
	nav_msgs::Path ORpath;
	nav_msgs::Path newpath;
	ORpath.header.stamp=ros::Time::now();
	ORpath.header.frame_id="/camera_init";
	newpath.header.stamp=ros::Time::now();
	newpath.header.frame_id="/camera_init";

	for(int j=0;j<global_path_.key_point.size();j++)
	{
			geometry_msgs::PoseStamped tempose;
		 	tempose.header.stamp=ORpath.header.stamp;
		 	tempose.header.frame_id=ORpath.header.frame_id;
		 	tempose.pose.position.x=global_path_.key_point[j].x;
		 	tempose.pose.position.y=global_path_.key_point[j].y;
		 	tempose.pose.position.z=0;
		 	ORpath.poses.push_back(tempose);
	}

	for(int i=0;i<Clothoid_arcfinderplanner_.m_refer_path.size();i++)
	{
		geometry_msgs::PoseStamped tempose;
		tempose.header.stamp = newpath.header.stamp;
		tempose.header.frame_id = newpath.header.frame_id;

		tempose.pose.position.x = Clothoid_arcfinderplanner_.
								  m_refer_path[i].x;
		tempose.pose.position.y = Clothoid_arcfinderplanner_.
								  m_refer_path[i].y;
		tempose.pose.position.z = 0;
		newpath.poses.push_back(tempose);

	}

	Key_point.publish(marker1);
	Node_text.publish(markerArray);
	orpath.publish(ORpath);
	smoothpath.publish(newpath);

}


int main(int argc, char** argv)
{

    ros::init(argc, argv, "path_planner");

    ros::NodeHandle node;

    g_pMapSubscriber = new ros::Subscriber;
    g_pPoseSubscriber = new ros::Subscriber;
    g_pclienter = new ros::ServiceClient;

    *g_pMapSubscriber = node.subscribe(strMapTopicName, 100, &HandleMapCallback);

    *g_pPoseSubscriber = node.subscribe(strPoseTopicName, 100, &HandlePoseCallback);

   
    *g_pclienter = node.serviceClient<Base_control::ControlCmd>("Controler");


	/// for clothpath
	pubPolgon = node.advertise<geometry_msgs::PolygonStamped> ("/Polygon",10);

	pubNodes = node.advertise<geometry_msgs::PoseArray>("/visualizeNodes3DPoses", 100);


	// robot and goal
	pubGoaler = node.advertise<geometry_msgs::PoseStamped> ("/Show_Goal",10);

	pubRobposer = node.advertise<geometry_msgs::PoseStamped> ("/Show_pose",10);;

/////////////////////////////// just for test ////////////////////////////////

 	Key_point = node.advertise<visualization_msgs::Marker>("Key_point",10);
 	Node_text = node.advertise<visualization_msgs::MarkerArray>("NodeID", 100);
    smoothpath = node.advertise<nav_msgs::Path>("smoothpath", 10);
    orpath = node.advertise<nav_msgs::Path>("orpath", 10);
 	

 	for(int i=0;i<200;i++)
 	{
 		std::string str;
 		str="show_path";
 		std::string str1;
 		std::stringstream stream;
 		stream<<i;
 		stream>>str1;
 		str=str+str1;

 		Mutraj_pub[i] = node.advertise<nav_msgs::Path>(str,1);

 	}

 	traj_pub = node.advertise<nav_msgs::Path>("chosen_path",1);

 	Conpointpuber = node.advertise<visualization_msgs::Marker>("controlpoint",1);


 	centerbiaozhun = node.advertise<nav_msgs::Path>("biaozhun",1);

 	MakerPosepub = node.advertise<visualization_msgs::Marker>("makerpose",1);

    ros::spin();
}

