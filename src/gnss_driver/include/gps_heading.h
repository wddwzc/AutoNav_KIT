/*
*
*  GNSS Headinga Analysis 
*  AUTHOR: He Guojian
*  2019-09
*/
#ifndef GPS_HEADINGA_H
#define GPS_HEADINGA_H

#include <string>
#include <vector>
#include <iostream>
#include <sstream>
#include <cstdio>
using namespace std;

//单选：使用的是哪种语句
#define USING_HEADINGA
//GPGGA sentence
//$GPGGA,121252.000,3937.3032,N,11611.6046,E,1,05,2.0,45.9,M,-5.7,M,,0000*77\r
class CHeading
{
public:
	string calculateStatus;		//<1> 解算状态   enum格式
	string locateStatus;		//<2> 定位状态	 enum格式
	float baseLine;				//<3> 基线长（0-3000米）
	float yawAngle ;			//<4> 航向角（0-360°）
	float pitchAngle;			//<5> 俯仰角（-90-90°）
	float yuLiu1;				//<6> 预留
	float yawAngleError;		//<7>航向角标准偏差
	float pitchAngleError;		//<8>俯仰角标准偏差
	string rtkID;				//<9>RTK基准站号
	int satelliteTrackNum;		//<10>参与跟踪的卫星数量
	int satelliteCalculateNum;	//<11>参与解算的卫星数量
	int satelliteHighNum;		//<12>截止高度角以上的卫星数量
	int satelliteL2Num;			//<13>截止高度角以上跟踪到L2的卫星颗数
	int yuLiu2;					//<14>预留
	int expandCalculateStatus;  //<15>扩展解算状态
	int yuLiu3;					//<16>预留
	int signalMaskStatus;    	//<17>信号掩码状态
	//int    bccVal; //校验
};

//处理heading语句的类
class CCopeHeading
{
public:
	int CHeadingData( char* );
	CCopeHeading();
	~CCopeHeading();

public:
	CHeading *p_heading;
	int num_new;  //当前处理完，产生了几条新语句
	
private:
	void SpliceData(char*);
	bool BccCheck( string& str );
	void ExtractData( string& str );

private:
	string raw_data;
	string a_frame;
};


CCopeHeading::CCopeHeading()
{
	raw_data = "";
	a_frame = "";
	p_heading = NULL;
#ifdef USING_HEADINGA
	p_heading = new CHeading[17];
#endif
	num_new = 0;
}
CCopeHeading::~CCopeHeading()
{
	if(p_heading) delete []p_heading;
}

//拼接字符串
void CCopeHeading::SpliceData(char* str_in) {
	raw_data += str_in;
	if(raw_data.size() > 4096)
	{
		raw_data = raw_data.substr(raw_data.size()-4096);
	}
}

bool CCopeHeading::BccCheck( string& str ){//"$...."
	//printf("bccCheck.\n");
	if (str.empty())
		return false;
	
	int a = str[1], i=2;
	while(str[i] != '*')
	{
		a ^= str[i];
		++i;
	}
	
	int    bccVal; //校验
	stringstream ss;
	ss << str[i+1] << str[i+2];
	ss >> hex >> bccVal;//hex
	ss.clear();
	if ( bccVal == a )
		return true;
	else
		return false;
}
//提取数据
void CCopeHeading::ExtractData( string& str )
{
	//std::cout<<"ExtractData!!!!!"<<std::endl;
	//protect the program, no more than 10 for once time!
	if(num_new > 10) return;
	
	//将字符串的分隔符由逗号变为空格（空内容则添加逗号），便于处理
	string str_copy;
	for (unsigned int i=0; i<str.size(); ++i)
	{
		//if(str[i] == ',' || str[i] == '*' || str[i] == ';')
		if(str[i] == ',' || str[i] == '*')
		{
				str_copy.push_back(' '); //更改为用“空格”做分隔符
				if(i>0 && str[i-1] == ',') //如果此项是空，用逗号代替实际内容
				{
					str_copy.push_back(','); 
					str_copy.push_back(' '); 
				}
		}
		else	
			str_copy.push_back(str[i]);
	}
	
#ifdef USING_HEADINGA
	//===============================================
	/*stringstream ss(str_copy);
	string tmp;
	
	//head：$HEADING3A
	for(int i = 0;i<11 ;i++)
		ss>>tmp;
	//ss >> tmp;*/
	int n = str_copy.find(';');
	string str_tmp = str_copy.substr(n);
	//std::cout<<"  "<<str_tmp<<std::endl;
	stringstream ss(str_tmp);
	
	string calculateStatus;		//<1> 解算状态   enum格式
	string locateStatus;		//<2> 定位状态	 enum格式
	float baseLine;				//<3> 基线长（0-3000米）
	float yawAngle ;			//<4> 航向角（0-360°）
	float pitchAngle;			//<5> 俯仰角（-90-90°）
	float yuLiu1;				//<6> 预留
	float yawAngleError;		//<7>航向角标准偏差
	float pitchAngleError;		//<8>俯仰角标准偏差
	string rtkID;				//<9>RTK基准站号
	int satelliteTrackNum;		//<10>参与跟踪的卫星数量
	int satelliteCalculateNum;	//<11>参与解算的卫星数量
	int satelliteHighNum;		//<12>截止高度角以上的卫星数量
	int satelliteL2Num;			//<13>截止高度角以上跟踪到L2的卫星颗数
	int yuLiu2;					//<14>预留
	int expandCalculateStatus;  //<15>扩展解算状态
	int yuLiu3;					//<16>预留
	int signalMaskStatus;    	//<17>信号掩码状态
	
	
	//<1> 解算状态   enum格式
	ss >> p_heading[num_new].calculateStatus;
	//<2> 定位状态	 enum格式
	ss >> p_heading[num_new].locateStatus;
	//<3> 基线长（0-3000米）
	ss >> p_heading[num_new].baseLine;
	//<4> 航向角（0-360°）
	ss >> p_heading[num_new].yawAngle;
	//<5> 俯仰角（-90-90°）
	ss >> p_heading[num_new].pitchAngle;
	//<6> 预留
	ss >> p_heading[num_new].yuLiu1;
	//<7>航向角标准偏差
	ss >> p_heading[num_new].yawAngleError;
	//<8>俯仰角标准偏差
	ss >> p_heading[num_new].pitchAngleError;
	//<9>RTK基准站号
	ss >> p_heading[num_new].rtkID;
	//<10>参与跟踪的卫星数量
	ss >> p_heading[num_new].satelliteTrackNum;
	//<11>参与解算的卫星数量
	ss >> p_heading[num_new].satelliteCalculateNum;
	//<12>截止高度角以上的卫星数量
	ss >> p_heading[num_new].satelliteHighNum;
	//<13>截止高度角以上跟踪到L2的卫星颗数
	ss >> p_heading[num_new].satelliteL2Num;
	//<14>预留
	ss >> p_heading[num_new].yuLiu2;
	//<15>扩展解算状态
	ss >> p_heading[num_new].expandCalculateStatus;
	//<16>预留
	ss >> p_heading[num_new].yuLiu3;
	//<17>信号掩码状态
	ss >> p_heading[num_new].signalMaskStatus;
	//===============================================
#endif
	//num + 1
	num_new++;
}



int CCopeHeading::CHeadingData( char* str_in )
{	
	//std::cout<<"Cheading data"<<std::endl;
	//set 0
	num_new = 0;
	//Splice with the old sentence
	SpliceData(str_in);  
	if(raw_data.size() < 20)
		return 0;
	
	//iteratively find a whole NEMA sentence, and analysis
	int pos_start = 0, pos_end = 0;
	while(pos_start!=string::npos && pos_end!=string::npos)
	{
		// find start
#ifdef USING_HEADINGA
		pos_start = raw_data.find("#HEADINGA", pos_end);
#endif
		if (pos_start == string::npos)
		{
			raw_data = raw_data.substr(pos_end+1);
			continue;
		}
		
		//find end
		pos_end = raw_data.find("\r", pos_start);
		if (pos_end == string::npos)
		{
			raw_data = raw_data.substr(pos_start);
			continue;
		}
		
		//get the whole sentence, and check
		a_frame.clear();
		a_frame = raw_data.substr(pos_start, pos_end-pos_start+1);
		/*if(!BccCheck(a_frame))  //check if ok!
		{
			//std::cout<<"check false!!!!!"<<std::endl;
			continue;
		}*/
		
		//analysis
		ExtractData(a_frame);
	}

	return num_new;
}


#endif


