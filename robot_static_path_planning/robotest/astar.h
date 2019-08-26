#pragma once  
/*
//A*�㷨������
*/
#include "basic.h"

const int kCost1 = 10; //ֱ��һ������  
const int kCost2 = 14; //б��һ������ 
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
	//bool IsDanger(MyPoint curp, MyPoint np, vector<vector<float>>& maze); //�жϻ�����ǰ���������Ƿ���Σ��
	bool IsBump(int* x_list, int *y_list, int a, int b, bool isdy); //�ж���������������֮���Ƿ����ϰ���
	void Release(list<MyPoint*> path);
	int th;
private:
	MyPoint *findPath(MyPoint &startPoint, MyPoint &endPoint, bool isIgnoreCorner);
	std::vector<MyPoint *> getSurroundPoints(const MyPoint *point, bool isIgnoreCorner) const;
	bool isCanreach(const MyPoint *point, const MyPoint *target, bool isIgnoreCorner) const; //�ж�ĳ���Ƿ����������һ���ж�  
	MyPoint *isInList(const std::list<MyPoint *> &list, const MyPoint *point) const; //�жϿ���/�ر��б����Ƿ����ĳ��  
	MyPoint *getLeastFpoint(); //�ӿ����б��з���Fֵ��С�Ľڵ�  
	int LimitRange(bool isdy, int x, int a, int b);
	//����FGHֵ  
	int calcG(MyPoint *temp_start, MyPoint *point);
	int calcH(MyPoint *point, MyPoint *end);
	int calcF(MyPoint *point);

	std::list<MyPoint *> openList;  //�����б�  
	std::list<MyPoint *> closeList; //�ر��б�  
};
#pragma once