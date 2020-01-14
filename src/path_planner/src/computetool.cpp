#include "path_planner/computetool.h"
#include "path_planner/Clothoid_arcfinder.h"
#include "path_planner/global_path.h"
#include "path_planner/Bspline_Mtrajfinder.h"
#include "path_planner/base.h"
#include "math.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PoseStamped.h"
#include <iostream>
#include <vector>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Quaternion.h>

#define pi  3.1415926535897

extern global_path global_path_;
extern Clothoid_arcfinder Clothoid_arcfinderplanner_;
extern Bspline_Mtrajfinder Bspline_Mtrajfinder_;

extern ros::Publisher pubPolgon;
extern std::vector<geometry_msgs::Point32> leftpolygon;
extern std::vector<geometry_msgs::Point32> rightpolygon;
extern ros::Publisher pubGoaler;
extern ros::Publisher pubRobposer;
extern ros::Publisher traj_pub;					//rviz上显示出最优轨迹的消息
extern ros::Publisher Mutraj_pub[200];
extern ros::Publisher Conpointpuber;

float length = 1.0;

float getLimitedRadius(float radius) {
	if (radius < 1.6) {
		radius = 1.6;
	}
	else if (radius > 10) {
		radius = 10;
	}
	return radius;
}

float toRadian(float degree)
{
	float radian = degree/180*pi;
	return radian;
     
}
float toDegree(float radian)
{

	  float degree = radian/pi*180;
	  degree=fmod(degree,360);
	  if(degree>=180)
	  {
	  degree=degree-360;
          return degree;
	  }
	  if(degree<-180)
	  {
	  degree=degree+360;
	   return degree;
	  }
	  return degree;
}

void generateplygon(const float &X,const float &Y,const float &TH,const float &CarR,const float &Carlength,const int &whichnode)
{
	float Expandlength = Carlength/2;

	float UnvectX = cos(TH);
	float UnvectY = sin(TH);
	
	float VeUnvectX = -sin(TH);
	float VeUnvectY = cos(TH);
	
	float Nedetath;
	float Podetath;
	float detath;
	
    detath = acos(UnvectX*VeUnvectX+UnvectY*VeUnvectY);
	
	Nedetath = TH - detath;
	Podetath = TH + detath;
	
	switch(whichnode)
	{
		case -1:
			{
				    geometry_msgs::Point32 Nepoint;
    				geometry_msgs::Point32 Popoint;

					Nepoint.x = X + cos(Nedetath)* CarR;
					Nepoint.y = Y + sin(Nedetath)* CarR;
	
					Popoint.x = X + cos(Podetath)* CarR;
					Popoint.y = Y + sin(Podetath)* CarR;
	
					leftpolygon.push_back(Popoint);
					rightpolygon.push_back(Nepoint);
		
					break;
			}
		case 1:
			{
				float back_X;
		        float back_Y;
		
		        back_X = X + cos(TH) * (-Expandlength);
	            back_Y = Y + sin(TH) * (-Expandlength);
		
				geometry_msgs::Point32 Nepoint;
    			geometry_msgs::Point32 Popoint;
		        geometry_msgs::Point32 back_Nepoint;
    			geometry_msgs::Point32 back_Popoint;
		
				back_Nepoint.x =  back_X + cos(Nedetath)* CarR;
				back_Nepoint.y =  back_Y + sin(Nedetath)* CarR;
	
				back_Popoint.x = back_X + cos(Podetath)* CarR;
				back_Popoint.y = back_Y + sin(Podetath)* CarR;
		
				leftpolygon.push_back(back_Popoint);
				rightpolygon.push_back(back_Nepoint);
		
		        Nepoint.x = X + cos(Nedetath)* CarR;
				Nepoint.y = Y + sin(Nedetath)* CarR;
	
				Popoint.x = X + cos(Podetath)* CarR;
				Popoint.y = Y + sin(Podetath)* CarR;
		
		        leftpolygon.push_back(Popoint);
				rightpolygon.push_back(Nepoint);
		
				break;
			}
		case 2:
			{
				float front_X;
		        float front_Y;
		  
		 	    front_X = X + cos(TH) * (Expandlength);
	            front_Y = Y + sin(TH) * (Expandlength);
		
		        geometry_msgs::Point32 Nepoint;
    			geometry_msgs::Point32 Popoint;
		        geometry_msgs::Point32 front_Nepoint;
    			geometry_msgs::Point32 front_Popoint;
		
				
				Nepoint.x = X + cos(Nedetath)* CarR;
				Nepoint.y = Y + sin(Nedetath)* CarR;
	
				Popoint.x = X + cos(Podetath)* CarR;
				Popoint.y = Y + sin(Podetath)* CarR;
	
				leftpolygon.push_back(Popoint);
				rightpolygon.push_back(Nepoint);
		
				front_Nepoint.x =  front_X + cos(Nedetath)* CarR;
				front_Nepoint.y =  front_Y + sin(Nedetath)* CarR;
	
				front_Popoint.x = front_X + cos(Podetath)* CarR;
				front_Popoint.y = front_Y + sin(Podetath)* CarR;
		
				leftpolygon.push_back(front_Popoint);
				rightpolygon.push_back(front_Nepoint);
	
				break;
			}
	}
}

void pubfianalpath(std::string &frame_id,std::vector<Node> &final_path)
{
	leftpolygon.clear();
	rightpolygon.clear();
	nav_msgs::Path path;
	int whichnode = -1;
	for(int j=final_path.size()-1;j>0;j--)
	{
		//std::cout<<"final path sample point 0.5m/point "<<std::endl;
		
		for(float l=0.0;l<=length;l=l+0.5)
		{
			geometry_msgs::PoseStamped tempose;

			if(frame_id == "/camera_init")
			{
				tempose.header.frame_id  = "/camera_init";
			}

			if(frame_id == "/rslidar")
			{
				tempose.header.frame_id  = "/rslidar";
			}
		    
		    tempose.header.stamp = ros::Time::now();
			float initCruv = final_path[j].Cruv;
			//float finalCruv = final_path[j-1].Cruv;
			float Cruv,Clothoid_Len;
			float X,Y,Th;
			if(l==0)
			{
				if(j==final_path.size()-1)
				{
					whichnode = 1;
				}
				X = final_path[j].x;
				Y = final_path[j].y;
				Th = final_path[j].Th;
			    Cruv = final_path[j].Cruv;
			}
			else
			{
				if(l==length && j==1)
				{
					whichnode = 2;
				}
				Node nodetemp;
				Clothoid_arcfinderplanner_.Gen_clothoid_arcP_XY(
					X,Y,Cruv,Clothoid_Len,final_path[j].x,
					final_path[j].y,final_path[j].Th,
					initCruv,final_path[j-1].Cruvrate,l);
				nodetemp.x = X;
				nodetemp.y = Y;
				nodetemp.Cruv = Cruv;
				nodetemp.Clothoid_Len = Clothoid_Len;
				nodetemp.parent = &final_path[j];
				Th = Clothoid_arcfinderplanner_.getNodeTh(
					&nodetemp,&final_path[j],final_path[j-1].Cruvrate,l);
			}
			
		   // if(l!=3.0)
		   // {
		    	//of1<<Cruv<<std::endl;
		    //}	

		    //std::cout<<"node>>>> "<<X<<" "<<Y<<" "<<Th<<" "<<Cruv<<" ";
			
			generateplygon(X,Y,Th,(Clothoid_arcfinderplanner_.Carwidth)/2,
						   (Clothoid_arcfinderplanner_.CarLength),whichnode);
			whichnode = -1;
			tempose.pose.position.x = X;
			tempose.pose.position.y = Y;
			path.poses.push_back(tempose);
		}
		std::cout<<std::endl;
	}
	
	geometry_msgs::PolygonStamped showpolgon;



	if(frame_id == "/camera_init")
	{
		showpolgon.header.frame_id =  "/camera_init";
	}

	if(frame_id == "/rslidar")
	{
		showpolgon.header.frame_id = "/rslidar";
	}

	
	showpolgon.header.stamp = ros::Time();
	
	for(int i=0;i<leftpolygon.size();i++)
	{
		showpolgon.polygon.points.push_back(leftpolygon[i]);
	}
	for(int j=rightpolygon.size()-1;j>=0;j--)
	{
		showpolgon.polygon.points.push_back(rightpolygon[j]);
	}
	pubPolgon.publish(showpolgon);
}



void pubGoal(std::string &frame_id,const float &goalx,const float &goaly,const float &goalth)
{
	geometry_msgs::PoseStamped GetgoalPose;
	GetgoalPose.header.stamp=ros::Time();
	GetgoalPose.header.frame_id = frame_id;
	GetgoalPose.pose.position.x = goalx;
	GetgoalPose.pose.position.y = goaly;
	GetgoalPose.pose.position.z =0;

	GetgoalPose.pose.orientation = tf::createQuaternionMsgFromYaw(goalth);
	

	// double x,y,z,w;				//姿态四元数
	// double yaw, pitch, roll; 
	// x = GetgoalPose.pose.orientation.x;  //注意，这里带着转换
	// y = GetgoalPose.pose.orientation.y;
	// z = GetgoalPose.pose.orientation.z;
	// w = GetgoalPose.pose.orientation.w;

	// tf::Matrix3x3(tf::Quaternion(x, y, z, w)).getRPY(roll, pitch, yaw);
	
	// std::cout<<"yaw "<<yaw<<" "<<pitch<<" "<<roll<<std::endl;

	// GetgoalPose.pose.orientation.x =0;
	// GetgoalPose.pose.orientation.y =0;
	// GetgoalPose.pose.orientation.z =sin(goalth/2);
	// GetgoalPose.pose.orientation.w =cos(goalth/2);
	pubGoaler.publish(GetgoalPose);
}

void pubRobpose(std::string &frame_id,const float &robx,const float &roby,
				const float &robz,const float &robrx,
				const float &robry,const float &robrz,const float &robrw,
				const float &robyaw)
{
		geometry_msgs::PoseStamped robotPose;
		robotPose.header.stamp=ros::Time();
		robotPose.header.frame_id = frame_id;
		robotPose.pose.position.x = robx;
		robotPose.pose.position.y = roby;
		robotPose.pose.position.z = 0.0;

		robotPose.pose.orientation = tf::createQuaternionMsgFromYaw(robyaw);

		

		// robotPose.pose.orientation.x = robrx;
		// robotPose.pose.orientation.y = robry;
		// robotPose.pose.orientation.x = robrz;
		// robotPose.pose.orientation.w = robrw;
		
		pubRobposer.publish(robotPose);

}


void pubClothoid_DcruvVelangTwist(const float &finalCruv,float &Clothoid_Dcruv,float linear,float twist)
{
	if((Clothoid_Dcruv ) * (Clothoid_Dcruv) < 0.01)
	{

		if((finalCruv-finalCruv) * (finalCruv-finalCruv)>0.01)
		{
			Clothoid_Dcruv = finalCruv;
		}
		else
		{
			linear = 0.8; //220
			twist = 0.0;
		}
	}

	std::cout<<"Clothoid_Dcruv ++ "<<Clothoid_Dcruv<<std::endl;

	if((Clothoid_Dcruv - 0.2) * (Clothoid_Dcruv - 0.2) < 0.01)
	{
		linear = 0.6; //175
		twist = 0.12;  //20
	}

	if((Clothoid_Dcruv + 0.2) * (Clothoid_Dcruv + 0.2) < 0.01)
	{
			linear = 0.6; //175
			twist = -0.12;  //20
	}

	if((Clothoid_Dcruv - 0.4) * (Clothoid_Dcruv - 0.4) < 0.01)
	{
			linear = 0.6; //175
			twist = 0.24; //65
	}

	if((Clothoid_Dcruv + 0.4) * (Clothoid_Dcruv + 0.4) < 0.01)
	{
			linear = 0.6; //175
			twist = -0.24; //65
	}

	if((Clothoid_Dcruv - 0.6) * (Clothoid_Dcruv - 0.6) < 0.01)
	{
			linear = 0.4; //130
			twist = 0.24; //200

	}

	if((Clothoid_Dcruv + 0.6) * (Clothoid_Dcruv + 0.6) < 0.01)
	{
			linear = 0.4; //130
			twist = -0.24; //200

	}

	if((Clothoid_Dcruv - 0.8) * (Clothoid_Dcruv - 0.8) < 0.01)
	{
			linear = 0.4;
			twist = 0.32;
							
	}

    if((Clothoid_Dcruv + 0.8) * (Clothoid_Dcruv + 0.8) < 0.01)
	{
		linear = 0.4;
		twist = -0.32;	
	}

	if((Clothoid_Dcruv - 1.0) * (Clothoid_Dcruv - 1.0) < 0.01)
	{
		linear = 0.2; //76
		twist = 0.2; //130
	}

	if((Clothoid_Dcruv + 1.0) * (Clothoid_Dcruv + 1.0) < 0.01)
	{
		linear = 0.2; //76
		twist = -0.2; //130
	}


	std::cout<<"Clothoid_Dcruv "<<Clothoid_Dcruv<<std::endl;
}

void pubtrajs(std::string &frame_id)
{
	std::vector<nav_msgs::Path> Mnowpath;
	for(int i=0;i<Bspline_Mtrajfinder_.CLbecheck_paths.size();i++)
	{
		nav_msgs::Path temnowpath;
		temnowpath.header.stamp=ros::Time::now();
		temnowpath.header.frame_id=frame_id;
		for(int j=0;j<Bspline_Mtrajfinder_.CLbecheck_paths[i].Ex_path.size();j++)
		{
			geometry_msgs::PoseStamped tempose;
			tempose.header.stamp=temnowpath.header.stamp;
			tempose.header.frame_id=temnowpath.header.frame_id;
			tempose.pose.position.x=Bspline_Mtrajfinder_.CLbecheck_paths[i].Ex_path[j].x;
			tempose.pose.position.y=Bspline_Mtrajfinder_.CLbecheck_paths[i].Ex_path[j].y;
			tempose.pose.position.z=0;
			temnowpath.poses.push_back(tempose);
		}
		Mnowpath.push_back(temnowpath);
	}
				
	for(int i=0;i<Bspline_Mtrajfinder_.Rbecheck_paths.size();i++)
	{
		nav_msgs::Path temnowpath;
		temnowpath.header.stamp=ros::Time::now();
		temnowpath.header.frame_id=frame_id;
		for(int j=0;j<Bspline_Mtrajfinder_.Rbecheck_paths[i].Ex_path.size();j++)
		{
			geometry_msgs::PoseStamped tempose;
			tempose.header.stamp=temnowpath.header.stamp;
			tempose.header.frame_id=temnowpath.header.frame_id;
			tempose.pose.position.x=Bspline_Mtrajfinder_.Rbecheck_paths[i].Ex_path[j].x;
			tempose.pose.position.y=Bspline_Mtrajfinder_.Rbecheck_paths[i].Ex_path[j].y;
			tempose.pose.position.z=0;
			temnowpath.poses.push_back(tempose);
		}
		Mnowpath.push_back(temnowpath);
	}

	/*-------------------rviz 中将最优路径用其他颜色重绘（rviz发送所有路径消息）-----------------*/
	std::cout<<"Bspline_Mtrajfinder_.pathID "<<Bspline_Mtrajfinder_.pathID<<"Bspline_Mtrajfinder_.CLnum "<<Bspline_Mtrajfinder_.CLnum<<std::endl;
	if(Bspline_Mtrajfinder_.pathID != -1)
	{
		nav_msgs::Path chosenpath;
		chosenpath.header.stamp=ros::Time::now();
		chosenpath.header.frame_id=frame_id;
		int chosennum=Bspline_Mtrajfinder_.pathID - Bspline_Mtrajfinder_.CLnum;
		if(chosennum<0)
		{
			for(int j=0;j<Bspline_Mtrajfinder_.RGJudgeOb_paths[Bspline_Mtrajfinder_.pathID].size();j++)
			{
				geometry_msgs::PoseStamped tempose;
				tempose.header.stamp=chosenpath.header.stamp;
				tempose.header.frame_id=chosenpath.header.frame_id;
				tempose.pose.position.x=Bspline_Mtrajfinder_.RGJudgeOb_paths[Bspline_Mtrajfinder_.pathID][j].x;
				tempose.pose.position.y=Bspline_Mtrajfinder_.RGJudgeOb_paths[Bspline_Mtrajfinder_.pathID][j].y;
				tempose.pose.position.z=0;
				// std::cout<<"final point "<<std::endl;
				// std::cout<<tempose.pose.position.x<<" "<<tempose.pose.position.y;
				chosenpath.poses.push_back(tempose);
			}
		}
		else
		{
			for(int j=0;j<Bspline_Mtrajfinder_.LGJudgeOb_paths[chosennum].size();j++)
			{
				geometry_msgs::PoseStamped tempose;
				tempose.header.stamp=chosenpath.header.stamp;
				tempose.header.frame_id=chosenpath.header.frame_id;
				tempose.pose.position.x=Bspline_Mtrajfinder_.LGJudgeOb_paths[chosennum][j].x;
				tempose.pose.position.y=Bspline_Mtrajfinder_.LGJudgeOb_paths[chosennum][j].y;
				tempose.pose.position.z=0;
				// std::cout<<"final point "<<std::endl;
				// std::cout<<tempose.pose.position.x<<" "<<tempose.pose.position.y;
				chosenpath.poses.push_back(tempose);
			}
		}
		// std::cout<<std::endl;
		traj_pub.publish(chosenpath);
	}


	for(int i=0;i<Mnowpath.size();i++)
	{
		Mutraj_pub[i].publish(Mnowpath[i]);
	}
}

std::vector<CPoint2d> Bezier(std::vector<CPoint2d> ctrlPoints,int n)  //BezierÇúÏß
{
	// std::cout<<"ctrlPoints size "<<ctrlPoints.size()<<std::endl;

 	std::vector<CPoint2d> Generatraj;

    float t, dt, t2, t3, f1, f2, f3, f4;
 
    dt = 1.0/n;      // t runs from 0 to 1.

  	for (t = 0.0; t <= 1.0; t += dt)
  	 {
         t2 = (1-t) *(1- t);
         t3 = t2 * (1-t);       // t3 =(1-t)*(1-t)*(1-t)
 
         f1 = t3;
         f2 = 3*t*t2;
         f3 = 3*t*t*(1-t);
         f4 = t*t*t;

         CPoint2d tmp;

         tmp.x = f1*ctrlPoints[0].x + f2*ctrlPoints[1].x + 
         			f3*ctrlPoints[2].x + f4*ctrlPoints[3].x;
 
         tmp.y = f1*ctrlPoints[0].y + f2*ctrlPoints[1].y + 
         		    f3*ctrlPoints[2].y + f4*ctrlPoints[3].y;

         Generatraj.push_back(tmp);   
     }
	 
	 for(int i=4;i<ctrlPoints.size();i++)
	 {
		 Generatraj.push_back(ctrlPoints[i]);
	 }

     return Generatraj;


}

void pubconpoint(std::string &frame_id,CPoint2d Conpint)
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = "/camera_init";
	marker.header.stamp = ros::Time::now();
	marker.ns = "/conp";
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.type =visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.scale.x = 0.1;
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;
	marker.lifetime = ros::Duration();
	marker.id =0;

	marker.pose.position.x=Conpint.x;
	marker.pose.position.y=Conpint.y;

	marker.pose.position.z=0.0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.color.a=1.0;
	marker.color.r = 0.0;
	marker.color.g = 0.0;
	marker.color.b = 0.0;
	Conpointpuber.publish(marker);
}

CPoint2d tansformWorldtoRob(CPoint2d &globalpoint,
									CPoint2d Rob)
{
	float dx,dy;
		
	dx = globalpoint.x - Rob.x;
	dy = globalpoint.y - Rob.y;
	
	CPoint2d subgoal;

	subgoal.x = dx * cos(Rob.pointangle) + dy * sin(Rob.pointangle);
	subgoal.y = dy * cos(Rob.pointangle) - dx * sin(Rob.pointangle);
	subgoal.pointangle = globalpoint.pointangle - Rob.pointangle;
	
	return subgoal;
}