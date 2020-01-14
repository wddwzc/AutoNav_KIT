#ifndef base_H
#define base_H

struct Node {
	float x;
	float y;
	float Cruv;	
    //Cruv rate;
	float Cruvrate; 
	float Th;
	float G;
	float H;
	float C;
	//pf-----//
	float pf;
	Node* parent;
	bool ol;
	bool cl;
	int obd;
  	//Clothoid_arc len
	float Clothoid_Len;
};

struct Node2D
{
	float x;
	float y;
    float G;
	float H; 
	Node2D* parent;
	bool ol;
	bool cl;
	int obd;
};


struct Executepoint
{
	bool SecondEx;
    int Firstconut;
    int Secondcount;
	int Pos;
	float Rotation;
};
#endif 
