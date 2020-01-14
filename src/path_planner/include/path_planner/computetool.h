#ifndef computetool_H
#define computetool_H
#include "path_planner/Clothoid_arcfinder.h"
#include "path_planner/base.h"
#include "math.h"
#include "geometry_msgs/Point32.h"
#include "path_planner/point.h"
#include <iostream>
#include <vector>
float getLimitedRadius(float radius);
float toRadian(float degree);
float toDegree(float radian);
void generateplygon(const float &X,const float &Y,const float &TH,const float &CarR,const float &Carlength,const int &whichnode);
void pubfianalpath(std::string &frame_id,std::vector<Node> &final_path);
void pubGoal(std::string &frame_id,const float &goalx,const float &goaly,const float &goalth);
void pubRobpose(std::string &frame_id,const float &robx,const float &roby,
				const float &robz,const float &robrx,
				const float &robry,const float &robrz,const float &robrw,
				const float &robyaw);
void pubClothoid_DcruvVelangTwist(const float &finalCruv,float &Clothoid_Dcruv,float linear,float twist);
void pubtrajs(std::string &frame_id);
void pubconpoint(std::string &frame_id,CPoint2d Conpint);
std::vector<CPoint2d> Bezier(std::vector<CPoint2d> ctrlPoints,int n);  //BezierÇúÏß
CPoint2d tansformWorldtoRob(CPoint2d &globalpoint,CPoint2d Rob);

#endif
