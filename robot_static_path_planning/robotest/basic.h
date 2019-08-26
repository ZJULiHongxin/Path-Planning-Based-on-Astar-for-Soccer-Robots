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
	int x, y; //�����꣬����Ϊ�˷��㰴��C++�����������㣬x������ţ�y��������  
	int F, G, H; //F=G+H  
	MyPoint *parent; //parent�����꣬����û����ָ�룬�Ӷ��򻯴���  
	MyPoint() {};
	MyPoint(int _x, int _y) :x(0), y(0), F(0), G(0), H(0), parent(NULL)  //������ʼ��  
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