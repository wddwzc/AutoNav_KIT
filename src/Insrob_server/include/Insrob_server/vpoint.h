#ifndef VPOINT_H
#define VPOINT_H

#include<iostream>

class VPoint
{
public:
	float x;
	float y;
	float z;

};

class VPointI : public VPoint
{
public:
	float intensity;  //0-255

};

class VPointRGB : public VPoint
{
public:
	unsigned char R;
	unsigned char G;
	unsigned char B;

};

class VPose
{
public:
	float x;
    float y;
    float z;
    
	float i;
    float j;
    float k;
	float w;
	
	float yaw;
	float pitch;
	float roll;
	
};

class VPoseStamped : public VPose
{
public:
    double timestamp;  
	
};

#endif // VPOINT_H
