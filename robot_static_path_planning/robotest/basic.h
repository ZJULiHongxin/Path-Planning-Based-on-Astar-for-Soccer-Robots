#pragma once
#include <iomanip> 
#include <iostream>
#include "Ws2tcpip.h"
#include "Winsock2.h"
#include"zss_cmd.pb.h"
#include"vision_detection.pb.h"
#include "zss_debug.pb.h"


#define CONTROLLED_CAR_ID 5

using std::cin;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;
using std::list;

struct MyPoint
{
	int x, y; //点坐标，这里为了方便按照C++的数组来计算，x代表横排，y代表竖列  
	int F, G, H; //F=G+H  
	MyPoint *parent; //parent的坐标，这里没有用指针，从而简化代码  
	MyPoint() {};
	MyPoint(int _x, int _y) :x(0), y(0), F(0), G(0), H(0), parent(NULL)  //变量初始化  
	{
		x = _x;
		y = _y;
	}
};

class VelPoint
{
public:
	VelPoint(){}

	double vx;
	double vy;
	double vr;
};

#pragma comment(lib, "ws2_32.lib")