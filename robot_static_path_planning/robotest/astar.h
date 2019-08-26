#pragma once  
/*
//A*算法对象类
*/
#include "basic.h"

const int kCost1 = 10; //直移一格消耗  
const int kCost2 = 14; //斜移一格消耗 
const int MaxAdditionalCost = 100;

MyPoint transp(int x, int y);
double inv_x(MyPoint p);
double inv_y(MyPoint p);



class Astar
{
public:
	void InitAstar();
	std::list<MyPoint*> GetPath(MyPoint &startPoint, MyPoint &endPoint, bool isIgnoreCorner);
	std::list<MyPoint> ShortCutting(std::list<MyPoint*>& longpath);
	int UpdateMap(Vision_DetectionFrame& Frame);
	std::vector<std::vector<float>> maze;
	//bool IsDanger(MyPoint curp, MyPoint np, vector<vector<float>>& maze); //判断机器人前进方向上是否有危险
	bool IsBump(int* x_list, int *y_list, int a, int b, bool isdy); //判断两个坐标点的连线之间是否有障碍物
	void Release(list<MyPoint*> path);
	int th;
private:
	MyPoint *findPath(MyPoint &startPoint, MyPoint &endPoint, bool isIgnoreCorner);
	std::vector<MyPoint *> getSurroundPoints(const MyPoint *point, bool isIgnoreCorner) const;
	bool isCanreach(const MyPoint *point, const MyPoint *target, bool isIgnoreCorner) const; //判断某点是否可以用于下一步判断  
	MyPoint *isInList(const std::list<MyPoint *> &list, const MyPoint *point) const; //判断开启/关闭列表中是否包含某点  
	MyPoint *getLeastFpoint(); //从开启列表中返回F值最小的节点  
	int LimitRange(bool isdy, int x, int a, int b);
	//计算FGH值  
	int calcG(MyPoint *temp_start, MyPoint *point);
	int calcH(MyPoint *point, MyPoint *end);
	int calcF(MyPoint *point);

	std::list<MyPoint *> openList;  //开启列表  
	std::list<MyPoint *> closeList; //关闭列表  
};
#pragma once