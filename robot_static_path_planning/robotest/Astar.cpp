/*----------------------------------------------------------------
// Copyright (C) 2019 李鸿鑫
// 版权所有。
//
// 文件名：astar.cpp
// 文件功能描述：实现A*路径规划，以及数字地图的更新
//
//
// 创建者：李鸿鑫(Hongxin Li)
// 时间：2019/8/26
//
// 版本：V1.0.0
//----------------------------------------------------------------*/

#include <math.h>  
#include "Astar.h"  
#include<iostream>
using namespace std;



  MyPoint transp(int x, int y) {
	  MyPoint temp(0, 0);
	  temp.x = (2250 - y) / 100;
	  temp.y = (x + 3000) / 100;
	  return temp;
  }

  //数字地图maze->实际地图
  double inv_x(MyPoint p) {
	  return (10 * p.y - 300);
  }

  double inv_y(MyPoint p) {
	  return (225 - 10 * p.x);
  }

  void Astar::InitAstar()
  {
	  
	  Release(openList);
	  openList.clear();
	  Release(closeList);
	  closeList.clear();
  }

  int Astar::calcG(MyPoint *temp_start, MyPoint *point)
  {
	  //int extraG = (abs(point->x - temp_start->x) + abs(point->y - temp_start->y)) == 1 ? kCost1 : kCost2;
	  int extraG = (abs(point->x - temp_start->x) + abs(point->y - temp_start->y)) == 1 ? (1+maze[point->x][point->y])*kCost1 : (1+maze[point->x][point->y]) * kCost2;
	  int parentG = point->parent == NULL ? 0 : point->parent->G; //如果是初始节点，则其父节点是空  
	  //cout << "Point的G：" << parentG + extraG<<"-------"<<endl;
	  return parentG + extraG;
  }

  int Astar::calcH(MyPoint *point, MyPoint *end)
  {

	  //用简单的欧几里得距离计算H，这个H的计算是关键 
	  return sqrt((double)(end->x - point->x)*(double)(end->x - point->x) + (double)(end->y - point->y)*(double)(end->y - point->y))*kCost1;
  }

  int Astar::calcF(MyPoint *point)
  {
	  return point->G + point->H;
  }

  MyPoint *Astar::getLeastFpoint()
  {
	  if (!openList.empty())
	  {
		  auto resPoint = openList.front();
		  for (auto &point : openList)
			  if (point->F < resPoint->F)
				  resPoint = point;
		  return resPoint;
	  }
	  return NULL;
  }

  MyPoint *Astar::findPath(MyPoint &startPoint, MyPoint &endPoint, bool isIgnoreCorner)
  {
	  openList.push_back(new MyPoint(startPoint.x, startPoint.y)); //置入起点,拷贝开辟一个节点，内外隔离  
	  while (!openList.empty())
	  {
		  auto curPoint = getLeastFpoint(); //找到F值最小的点  
		  openList.remove(curPoint); //从开启列表中删除  
		  closeList.push_back(curPoint); //放到关闭列表  
		  //1,找到当前周围八个格中可以通过的格子  
		  auto surroundPoints = getSurroundPoints(curPoint, isIgnoreCorner);

		  for (auto &target : surroundPoints)
		  {
			  //2,对某一个格子，如果它不在开启列表中，加入到开启列表，设置当前格为其父节点，计算F G H  
			  if (!isInList(openList, target))
			  {
				  target->parent = curPoint;

				  target->G = calcG(curPoint, target);
				  target->H = calcH(target, &endPoint);
				  target->F = calcF(target);

				  openList.push_back(target);
			  }
			  //3，对某一个格子，它在开启列表中，计算G值, 如果比原来的大, 就什么都不做, 否则设置它的父节点为当前点,并更新G和F  
			  else
			  {
				  int tempG = calcG(curPoint, target);
				  if (tempG < target->G)
				  {
					  target->parent = curPoint;

					  target->G = tempG;
					  target->F = calcF(target);
				  }
			  }
			  MyPoint *resPoint = isInList(openList, &endPoint);
			  if (resPoint)
				  return resPoint; //返回列表里的节点指针，不要用原来传入的endpoint指针，因为发生了深拷贝  
		  }

	  }

	  return NULL;
  }

  std::list<MyPoint*> Astar::GetPath(MyPoint &startPoint, MyPoint &endPoint, bool isIgnoreCorner)
  {
	  MyPoint *result = findPath(startPoint, endPoint, isIgnoreCorner);
	  std::list<MyPoint*> path;
	  //返回路径，如果没找到路径，返回空链表  
	  while (result)
	  {
		  path.push_front(result);
		  result = result->parent;
	  }
	  return path;
  }

  MyPoint *Astar::isInList(const std::list<MyPoint *> &list, const MyPoint *point) const
  {
	  //判断某个节点是否在列表中，这里不能比较指针，因为每次加入列表是新开辟的节点，只能比较坐标  
	  for (auto p : list)
		  if (p->x == point->x&&p->y == point->y)
			  return p;
	  return NULL;
  }

  bool Astar::isCanreach(const MyPoint *point, const MyPoint *target, bool isIgnoreCorner) const
  {
	  if (target->x<0 || target->x>maze.size() - 1
		  || target->y<0 && target->y>maze[0].size() - 1
		  || maze[target->x][target->y] ==100
		  || target->x == point->x&&target->y == point->y
		  || isInList(closeList, target)) //如果点与当前节点重合、超出地图、是障碍物、或者在关闭列表中，返回false  
		  return false;
	  else
	  {
		  if (abs(point->x - target->x) + abs(point->y - target->y) == 1) //非斜角可以  
			  return true;
		  else
		  {
			  //斜对角要判断是否绊住  
			  if (maze[point->x][target->y] == 0 && maze[target->x][point->y] == 0)
				  return true;
			  else
				  return isIgnoreCorner;
		  }
	  }
  }

  std::vector<MyPoint *> Astar::getSurroundPoints(const MyPoint *point, bool isIgnoreCorner) const
  {
	  std::vector<MyPoint *> surroundPoints;

	  for (int x = point->x - 1; x <= point->x + 1; x++)
		  for (int y = point->y - 1; y <= point->y + 1; y++)
			  if (isCanreach(point, new MyPoint(x, y), isIgnoreCorner))
				  surroundPoints.push_back(new MyPoint(x, y));

	  return surroundPoints;
  }

  std::list<MyPoint> Astar::ShortCutting(std::list<MyPoint *>& longpath)
  {
	  //橡皮筋算法三
	  const int node_count = longpath.size();
	  cout << "路径长度" << node_count << endl;

	  //获取原始路径中的转折点
	  list<MyPoint*> temppath;

	  int last_x = longpath.front()->x, last_y = longpath.front()->y;
	  int last_x_diff = 0, last_y_diff = 0;
	  int cur_x, cur_y;
	  list<MyPoint*>::iterator iter;
	  for (iter = (++longpath.begin()); iter != longpath.end(); iter++)
	  {
		  //如果当前两点斜率不同于上一点斜率，则把上一点当作转折点
		  cur_x = (*iter)->x;
		  cur_y = (*iter)->y;
		  if (cur_y - last_y != last_y_diff || cur_x - last_x != last_x_diff)
		  {
			  temppath.push_back(*(--iter));
			  iter++;
		  }
		  last_x_diff = cur_x - last_x;
		  last_y_diff = cur_y - last_y;
		  last_x = cur_x;
		  last_y = cur_y;
	  }
	  temppath.push_back(*(--iter));

	  int len = temppath.size();

	  int* path_point_x = new int[len];
	  int* path_point_y = new int[len];
	  int i = 0;

	  for (iter = temppath.begin(); iter != temppath.end(); iter++)
	  {
		  path_point_x[i] = (*iter)->x;
		  path_point_y[i++] = (*iter)->y;
	  }
	  list<MyPoint> final_path;
	  int nextend;

	  final_path.push_back(*(longpath.front()));
	  i = 0;
	  while (1)
	  {
		  int k;
		  nextend = i + 1;
		  for (k = len - 1; k > i + 1; k--)
		  {
			  if (IsBump(path_point_x, path_point_y, i, k, false) == false)
			  {
				  nextend = k;
				  break;
			  }
		  }
		  final_path.push_back(MyPoint(path_point_x[nextend], path_point_y[nextend]));
		  i = nextend;

		  if (i == len - 2)
		  {
			  final_path.push_back(MyPoint(path_point_x[len - 1], path_point_y[len - 1]));
			  break;
		  }
		  if (i == len - 1)
		  {
			  break;
		  }
	  }

	  delete[]path_point_x;
	  delete[]path_point_y;

	  return final_path;
  }

  //std::list<MyPoint> Astar::ShortCutting(std::list<MyPoint *>& longpath, vector<vector<float>>&maze)
  //{
	 // const int node_count = longpath.size();
	 // //cout << "路径长度" << node_count << endl;
	 // int* coord_x = new int[node_count];
	 // int* coord_y = new int[node_count];
	 // list<MyPoint*>::iterator iter;

	 // bool debug_drawmap = 0;
	 // bool debug_printpath =false;

	 // int path_len = 0;

	 // if (debug_printpath == true)
	 // {
		//  cout << "原始路径" << endl;
	 // }

	 // //debug_drawmap
	 // if (debug_drawmap == true)
	 // {
		//  for (int i = 0; i < 46; i++)
		//  {
		//	  for (int j = 0; j < 61; j++)
		//	  {
		//		  cout << int(maze[i][j]) << ' ';
		//	  }
		//	  cout << endl;
		//  }
	 // }
	 // //debug_drawmap

	 // for (iter = longpath.begin(); iter != longpath.end(); iter++)
	 // {
		//  //debug_drawmap
		//  if (debug_drawmap == true)
		//  {
		//	  maze[(*iter)->x][(*iter)->y] = 7;
		//  }
		//  if (debug_printpath == true)
		//  {
		//	  cout << '(' << (*iter)->x << ',' << (*iter)->y << ')' << endl;
		//  }

		//  coord_x[path_len] = (*iter)->x;
		//  coord_y[path_len] = (*iter)->y;
		//  path_len++;
	 // }



	 // if (debug_printpath == true)
	 // {
		//  cout << "获取原始路径中的转折点" << endl;
	 // }

	 // //获取原始路径中的转折点
	 // int * turning_point_x = new int[node_count];
	 // int * turning_point_y = new int[node_count];
	 // int j = 0;
	 // int last_x = coord_x[0], last_y = coord_y[0];
	 // int last_x_diff = 0, last_y_diff = 0;

	 // for (int i = 1; i < node_count; i++)
	 // {
		//  //如果当前两点斜率不同于上一点斜率，则把上一点当作转折点
		//  if (coord_y[i] - last_y != last_y_diff || coord_x[i] - last_x != last_x_diff)
		//  {
		//	  turning_point_x[j] = coord_x[i - 1]; turning_point_y[j] = coord_y[i - 1];
		//	  j++;
		//  }
		//  last_x_diff = coord_x[i] - last_x;
		//  last_y_diff = coord_y[i] - last_y;
		//  last_x = coord_x[i];
		//  last_y = coord_y[i];
	 // }
	 // turning_point_x[j] = coord_x[node_count - 1]; turning_point_y[j] = coord_y[node_count - 1];
	 // j++;

	 // if (debug_printpath == true)
	 // {
		//  for (int i = 0; i < j; i++)
		//  {
		//	  cout << '(' << turning_point_x[i] << ',' << turning_point_y[i] << ')' << endl;
		//  }
	 // }

	 // //橡皮筋算法一
	 // //int* keypoint_x = new int[j];
	 // //int* keypoint_y = new int[j];
	 // //keypoint_x[0] = turning_point_x[0];
	 // //keypoint_y[0] = turning_point_y[0];
	 // //int keypoint_count = 1;
	 // //for (int i = 0; i < j - 1; )
	 // //{
		// // int k;
		// // for (k = i + 1; k < j; k++)
		// // {
		//	//  if (IsBump(turning_point_x, turning_point_y, i, k, maze, false) == true)
		//	//  {
		//	//	  keypoint_x[keypoint_count] = turning_point_x[k - 1];
		//	//	  keypoint_y[keypoint_count] = turning_point_y[k - 1];
		//	//	  keypoint_count++;
		//	//	  break;
		//	//  }
		// // }
		// // i = k - 1;
	 // //}
	 // //keypoint_x[keypoint_count] = turning_point_x[j - 1];
	 // //keypoint_y[keypoint_count] = turning_point_y[j - 1];
	 // //keypoint_count++;

	 // //橡皮筋算法三
	 // int* keypoint_x = new int[j];
	 // int* keypoint_y = new int[j];
	 // keypoint_x[0] = turning_point_x[0];
	 // keypoint_y[0] = turning_point_y[0];

	 // int keypoint_count = 1;
	 // int i=0;
	 // int nextend;
	 // while (1)
	 // {
		//  int k;
		//  nextend = i + 1;
		//  //cout << "jjjjjjjjj:" << j << endl;
		//  for (k = j-1; k > i+1; k--)
		//  {
		//	  //cout << "isbump-------------->" << endl;
		//	  if (IsBump(turning_point_x, turning_point_y, i, k, maze, false)==false)
		//	  {
		//		  nextend = k;
		//		  break;
		//	  }
		//	  //cout << ">>>>>>>>>>>>>>>>over>>>>" << endl;
		//  }
		// // cout << ">>>>>>>>>>>>>>>>end for >>>>" << endl;
		//  keypoint_x[keypoint_count] = turning_point_x[nextend];
		//  keypoint_y[keypoint_count] = turning_point_y[nextend];
		//  keypoint_count++;
		//  i = nextend;
		//  //cout << "i:" << i << '\t';
		//  if (i == j - 2)
		//  {
		//	  keypoint_x[keypoint_count] = turning_point_x[j-1];
		//	  keypoint_y[keypoint_count] = turning_point_y[j-1];
		//	  keypoint_count++;
		//	  break;
		//  }
		//  if (i == j - 1)
		//  {
		//	  break;
		//  }
	 // }
	 // 
	 // 

	 // /*if (debug_printpath)
	 // {
		//  cout << "得到最终路径" << endl;
	 // }*/


	 // list<MyPoint> Final_path;

	 // for (int i = 0; i < keypoint_count; i++)
	 // {
		//  if (debug_printpath)
		//  {
		//	  cout << '(' << keypoint_x[i] << ',' << keypoint_y[i] << ')' << endl;
		//	  maze[keypoint_x[i]][keypoint_y[i]] = 9;
		//  }

		//  //debug_drawmap
		//  
		//  //debug_drawmap
		//  Final_path.push_back(MyPoint(keypoint_x[i], keypoint_y[i]));
	 // }

	 // //debug_drawmap
	 // if (debug_drawmap == true)
	 // {
		//  for (int i = 0; i < 46; i++)
		//  {
		//	  for (int j = 0; j < 61; j++)
		//	  {
		//		  cout << int(maze[i][j]) << ' ';
		//	  }
		//	  cout << endl;
		//  }
	 // }

	 // //debug_drawmap

	 // delete[]coord_x;
	 // delete[]coord_y;
	 // delete[]turning_point_x;
	 // delete[]turning_point_y;
	 // delete[]keypoint_x;
	 // delete[]keypoint_y;

	 // return Final_path;

  //}

  int Astar::LimitRange(bool isdy, int x, int a, int b) {
	  if (isdy)		//isdy=1, 动态避障，则限制范围
		  return max(a, min(b, abs(x)));
	  else        //isdy=0, 拉直路径，不限制范围
		  return abs(x);
  }

  bool Astar::IsBump(int* x_list, int *y_list, int a, int b, bool isdy)
  {
	  //cout << "判断" << '(' << x_list[a] << ',' << y_list[a] << ')' << "和" << '(' << x_list[b] << ',' << y_list[b] << ')' << "之间是否有障碍物" << endl;
	  bool flag = false;
	  double x = x_list[a], y = y_list[a];

	  if ((y_list[b] - y_list[a]) != 0 && (x_list[b] - x_list[a]) != 0)
	  {
		  if (abs(y_list[b] - y_list[a]) > abs(x_list[b] - x_list[a]))
		  {

			  double gradient = abs(double(x_list[b] - x_list[a]) / double(y_list[b] - y_list[a]));
			  int x_step = x_list[b] > x_list[a] ? 1 : -1;
			  int y_step = y_list[b] > y_list[a] ? 1 : -1;

			  x = x + gradient * x_step;
			  y = y + y_step;
			  for (int count = 0; count < LimitRange(isdy, y_list[a] - y_list[b], 3, 9); count++)
			  {
				  //if (maze[(int)x][(int)y] == 1 || maze[(int)x][(int)(y + 1)] == 1 || maze[(int)x][(int)(y - 1)] == 1)
				  //cout << '(' << round(x) << ',' << round(y) << ')' << ' '<< maze[round(x)][round(y)]<<endl;
				  x = x + gradient * x_step;
				  y = y + y_step;
				  if (maze[round(x)][round(y+y_step)]>=th &&
					  maze[round(x)][round(y) + 1] >=th &&
					  maze[round(x) + 1][round(y)] >=th &&
					  maze[round(x) - 1][round(y)] >=th &&
					  maze[round(x)][round(y) - 1] >=th)
				  {

					  flag = true;
					  //cout << "有障碍物" << endl;
					  break;
				  }
				  
			  }
		  }
		  else if (abs(y_list[b] - y_list[a]) < abs(x_list[b] - x_list[a]))
		  {


			  double gradient = abs(double(y_list[b] - y_list[a]) / double(x_list[b] - x_list[a]));
			  int x_step = x_list[b] > x_list[a] ? 1 : -1;
			  int y_step = y_list[b] > y_list[a] ? 1 : -1;

			  x = x + gradient * x_step;
			  y = y + y_step;
			  for (int count = 0; count < LimitRange(isdy, x_list[b] - x_list[a], 3, 9); count++)
			  {
				  //if (maze[(int)x][(int)y] == 1 || maze[(int)x][(int)(y + 1)] == 1 || maze[(int)x][(int)(y - 1)] == 1)
				  x = x + x_step;
				  y = y + y_step * gradient;

				  if (maze[round(x)][round(y)] >=th &&
					  maze[round(x)][round(y) + 1] >=th &&
					  maze[round(x) + 1][round(y)] >=th &&
					  maze[round(x) - 1][round(y)] >=th &&
					  maze[round(x)][round(y) - 1] >=th)
				  {
					  flag = true;
					  //cout << "有障碍物" << endl;
					  break;
				  }
				  
			  }
		  }
	  }
	  else if ((y_list[b] - y_list[a]) == 0)
	  {
		  int x_step = x_list[b] > x_list[a] ? 1 : -1;

		  x += x_step;
		  for (int count = 0; count < LimitRange(isdy, x_list[b] - x_list[a], 3, 9); count++)
		  {
			  x += x_step;
			  if (maze[x][y_list[b]] >= th &&
				  maze[round(x)][round(y_list[b]) + 1] >=th &&
				  maze[round(x) + 1][round(y_list[b])] >=th &&
				  maze[round(x) - 1][round(y_list[b])] >=th &&
				  maze[round(x)][round(y_list[b]) - 1] >=th)
			  {
				  flag = true;
				  //cout << "有障碍物" << endl;
				  break;
			  }
			  
		  }
	  }
	  else if ((x_list[b] - x_list[a]) == 0)
	  {
		  int y_step = y_list[b] > y_list[a] ? 1 : -1;
		  y += y_step;
		  for (int count = 0; count < LimitRange(isdy, y_list[b] - y_list[a],3, 9); count++)
		  {
			  y += y_step;
			  if (maze[x_list[a]][y] >= th &&
				  maze[round(x_list[a])][round(y) + 1] >=th &&
				  maze[round(x_list[a]) + 1][round(y)] >=th &&
				  maze[round(x_list[a]) - 1][round(y)] >=th &&
				  maze[round(x_list[a])][round(y) - 1] >=th)
			  {
				  flag = true;
				  //cout << "有障碍物" << endl;
				  break;
			  }
			  
		  }
	  }
	  return flag;
  }

//bool Astar::IsDanger(MyPoint curp, MyPoint np, vector<vector<float>>& maze)
//{
//	bool debug_isbump = false;
//
//	double curp_x = inv_x(curp);
//	double curp_y = inv_y(curp);
//	double np_x = inv_x(np);
//	double np_y = inv_y(np);
//
//	if (debug_isbump == true)
//	{
//		cout << '\n' << "判断实际坐标:" << '(' << curp_x << ',' << curp_y << ')' << "和" << '(' << np_x << ',' << np_y << ')' << "之间是否危险" << endl;
//	}
//
//
//	bool flag = false;
//
//	double a = np_y - curp_y;
//	double b = curp_x - np_x;
//	double c = np_y * curp_x - np_x * curp_y;
//	double length = sqrt((curp_y - np_y)*(curp_y - np_y) + (curp_x - np_x)*(curp_x - np_x));
//
//	int x_step;
//	int y_step;
//
//	if (a > 0) y_step = 10;
//	else if (a == 0) y_step = 0;
//	else if (a < 0) y_step = -10;
//
//	if (b < 0) x_step = 10;
//	else if (b == 0) x_step = 0;
//	else if (b > 0) x_step = -10;
//
//	for (int i = 1; i < 4; i++)
//	{
//		for (int j = 1; j < 4; j++)
//		{
//
//			if (curp_x + i * x_step >= -300 && curp_x + i * x_step <= 300 && curp_y + j * y_step >= -225 && curp_y + j * y_step <= 225)
//			{
//				double result = abs(a * (curp_x + i * x_step) + b * (curp_x + i * x_step) + c) / length;
//
//				if (debug_isbump == true)
//				{
//					cout << '(' << curp_x + i * x_step << ',' << curp_y + j * y_step << ')' << "是否在范围内"
//						<< "该点到前进方向直线的距离是" << result << endl;
//				}
//
//
//
//				MyPoint temp = transp(curp_x + i * x_step, curp_y + j * y_step);
//				if (result <= 300)
//				{
//
//					//cout << "判断该路径上的" << '(' << curp_x + i * x_step << ',' << curp_y + j * y_step << ')' << "的危险度" << endl;
//					if (debug_isbump == true)
//					{
//						cout << '(' << curp_x + i * x_step << ',' << curp_y + j * y_step << ')' << "是否在范围内"
//							<< "该点到前进方向直线的距离是" << result << endl;
//						cout << "该路径上的" << '(' << curp_x + i * x_step << ',' << curp_y + j * y_step << ')'
//							<< "即" << '(' << temp.x << ',' << temp.y << ')' << "的危险度是" << maze[temp.x][temp.y] << endl;
//					}
//
//					if (maze[temp.x][temp.y] > 0.2 && maze[temp.x][temp.y] < 1)
//					{
//						cout << "该路径上的" << '(' << curp_x + i * x_step << ',' << curp_y + j * y_step << ')'
//							<< "即" << '(' << temp.x << ',' << temp.y << ')' << "的危险度是" << maze[temp.x][temp.y] << endl;
//						flag = true;
//						break;
//					}
//				}
//			}
//
//
//		}
//	//	if (flag == true) break;
//	//}
//
//	if (debug_isbump == true)
//	{
//		if (flag == true) cout << " 有危险" << endl;
//		else cout << " 安全" << endl;
//	}
//
//
//	return flag;
//}

  //int Astar::UpdateMap(Vision_DetectionFrame& Frame)
  //{
	 // MyPoint occupation(0, 0);
	 // int bluenum = Frame.robots_blue_size();
	 // int yellownum = Frame.robots_yellow_size();
	 // int ID;

	 // //重置地图，将除了边界点以外的点重置为0
	 // for (int i = 1; i <= 45; i++)
	 // {
		//  maze[i][0] = 1.0;
		//  maze[i][61] = 1.0;
	 // }
	 // for (int i = 1; i <= 60; i++)
	 // {
		//  maze[0][i] = 1.0;
		//  maze[46][i] = 1.0;
	 // }
	 // for (int i = 1; i <= 45; i++)
	 // {
		//  for (int j = 1; j <= 60; j++)
		//  {
		//	  maze[i][j] = 0.;
		//  }
	 // }


	 // //根据场上障碍物更新地图
	 // for (int i = 0; i < yellownum; i++)
	 // {
		//  occupation = transp(Frame.robots_yellow(i).x(), Frame.robots_yellow(i).y());  //障碍物坐标
		//  float occu_vel_x = Frame.robots_yellow(i).raw_vel_x();   //障碍物x方向全局速度
		//  float occu_vel_y = Frame.robots_yellow(i).raw_vel_y();   //障碍物y方向全局速度

		//  //速度比例
		//  float x_ration = occu_vel_x / 100.0;
		//  float y_ration = occu_vel_y / 100.0;

		//  if (occupation.x >= 1 && occupation.x <= 44 && occupation.y >= 1 && occupation.y <= 59)
		//  {
		//	  //膨胀周围两圈
		//	  maze[occupation.x][occupation.y] = 1;
		//	  maze[occupation.x + 1][occupation.y] = 1;
		//	  maze[occupation.x - 1][occupation.y] = 1;
		//	  maze[occupation.x][occupation.y + 1] = 1;
		//	  maze[occupation.x][occupation.y - 1] = 1;

		//	  maze[occupation.x + 1][occupation.y + 1] = 1;
		//	  maze[occupation.x - 1][occupation.y + 1] = 1;
		//	  maze[occupation.x + 1][occupation.y - 1] = 1;
		//	  maze[occupation.x - 1][occupation.y - 1] = 1;

		//	  /*maze[occupation.x + 2][occupation.y + 2] = 1;
		//	  maze[occupation.x + 2][occupation.y + 1] = 1;
		//	  maze[occupation.x + 2][occupation.y] = 1;
		//	  maze[occupation.x + 2][occupation.y - 1] = 1;
		//	  maze[occupation.x + 2][occupation.y - 2] = 1;

		//	  maze[occupation.x + 1][occupation.y + 2] = 1;
		//	  maze[occupation.x][occupation.y + 2] = 1;
		//	  maze[occupation.x - 1][occupation.y + 2] = 1;

		//	  maze[occupation.x - 2][occupation.y + 2] = 1;
		//	  maze[occupation.x - 2][occupation.y + 1] = 1;
		//	  maze[occupation.x - 2][occupation.y] = 1;
		//	  maze[occupation.x - 2][occupation.y - 1] = 1;
		//	  maze[occupation.x - 2][occupation.y - 2] = 1;

		//	  maze[occupation.x + 1][occupation.y - 2] = 1;
		//	  maze[occupation.x][occupation.y - 2] = 1;
		//	  maze[occupation.x - 1][occupation.y - 2] = 1;*/

		//	  //将障碍物移动方向上前面几格也设置为障碍物
		//	  //for (int i = -2; i <= 2; i++)
		//	  //{
		//		 // if (x_ration > 0)
		//		 // {
		//			//  maze[occupation.x + i][occupation.y + 3] = abs(x_ration) / 10.0;
		//			//  maze[occupation.x + i / 2][occupation.y + 4] = abs(x_ration) / 20.0;
		//		 // }
		//		 // if (x_ration < 0)
		//		 // {
		//			//  maze[occupation.x + i][occupation.y - 3] = abs(x_ration) / 10.0;
		//			//  maze[occupation.x + i / 2][occupation.y + 4] = abs(x_ration) / 20.0;
		//		 // }
		//	  //}

		//	  //for (int i = -2; i <= 2; i++)
		//	  //{
		//		 // if (y_ration > 0)
		//		 // {
		//			//  maze[occupation.x + 3][occupation.y + i / 2] = abs(y_ration) / 10.0;
		//			//  maze[occupation.x + 4][occupation.y + i / 2] = abs(y_ration) / 20.0;
		//		 // }
		//		 // if (y_ration < 0)
		//		 // {
		//			//  maze[occupation.x - 3][occupation.y + i / 2] = abs(y_ration) / 10.0;
		//			//  maze[occupation.x + 4][occupation.y + i / 2] = abs(y_ration) / 20.0;
		//		 // }
		//	  //}

		//  }

	 // }
	 // for (int i = 0; i < bluenum; i++)
	 // {
		//  occupation = transp(Frame.robots_blue(i).x(), Frame.robots_blue(i).y());
		//  float occu_vel_x = Frame.robots_blue(i).raw_vel_x();   //障碍物x方向全局速度
		//  float occu_vel_y = Frame.robots_blue(i).raw_vel_y();   //障碍物y方向全局速度

		//  //速度比例
		//  float x_ration = occu_vel_x / 100.0;
		//  float y_ration = occu_vel_y / 100.0;

		//  if (Frame.robots_blue(i).robot_id() != CONTROLLED_CAR_ID) {
		//	  if (occupation.x >= 1 && occupation.x <= 45 && occupation.y >= 1 && occupation.y <= 45)
		//	  {
		//		  occupation = transp(Frame.robots_blue(i).x(), Frame.robots_blue(i).y());
		//		  maze[occupation.x][occupation.y] = 1;
		//		  maze[occupation.x + 1][occupation.y] = 1;
		//		  maze[occupation.x - 1][occupation.y] = 1;
		//		  maze[occupation.x][occupation.y + 1] = 1;
		//		  maze[occupation.x][occupation.y - 1] = 1;

		//		  maze[occupation.x + 1][occupation.y + 1] = 1;
		//		  maze[occupation.x - 1][occupation.y + 1] = 1;
		//		  maze[occupation.x + 1][occupation.y - 1] = 1;
		//		  maze[occupation.x - 1][occupation.y - 1] = 1;

		//	  /*maze[occupation.x + 2][occupation.y + 2] = 1;
		//		  maze[occupation.x + 2][occupation.y + 1] = 1;
		//		  maze[occupation.x + 2][occupation.y] = 1;
		//		  maze[occupation.x + 2][occupation.y - 1] = 1;
		//		  maze[occupation.x + 2][occupation.y - 2] = 1;

		//		  maze[occupation.x + 1][occupation.y + 2] = 1;
		//		  maze[occupation.x][occupation.y + 2] = 1;
		//		  maze[occupation.x - 1][occupation.y + 2] = 1;

		//		  maze[occupation.x - 2][occupation.y + 2] = 1;
		//		  maze[occupation.x - 2][occupation.y + 1] = 1;
		//		  maze[occupation.x - 2][occupation.y] = 1;
		//		  maze[occupation.x - 2][occupation.y - 1] = 1;
		//		  maze[occupation.x - 2][occupation.y - 2] = 1;

		//		  maze[occupation.x + 1][occupation.y - 2] = 1;
		//		  maze[occupation.x][occupation.y - 2] = 1;
		//		  maze[occupation.x - 1][occupation.y - 2] = 1;*/
		//	  }
		//  }
		//  else if (Frame.robots_blue(i).robot_id() == CONTROLLED_CAR_ID) {
		//	  ID = i;
		//  }
	 // }
	 // //cout << "Update Map Success!" << endl;
	 // return ID;

  //}

  int Astar::UpdateMap(Vision_DetectionFrame& Frame)
  {
	  //cout << "g" << endl;
	  MyPoint occupation(0, 0);
	  int bluenum = Frame.robots_blue_size();
	  int yellownum = Frame.robots_yellow_size();
	  int ID;


	  //重置地图，将除了边界点以外的点重置为0
	  for (int i = 0; i <= 45; i++)
	  {
		  maze[i][0] = 100.0;
		  maze[i][60] = 100.0;
	  }
	  for (int i = 1; i <= 59; i++)
	  {
		  maze[0][i] = 100.0;
		  maze[45][i] = 100.0;
	  }
	  for (int i = 1; i <= 44; i++)
	  {
		  for (int j = 1; j <= 59; j++)
		  {
			  maze[i][j] = 0.;
		  }
	  }


	  //根据场上障碍物更新地图
	  for (int i = 0; i < yellownum; i++)
	  {
		  occupation = transp(Frame.robots_yellow(i).x(), Frame.robots_yellow(i).y());  //障碍物坐标

		  if (occupation.x >= 1 && occupation.x <= 44 && occupation.y >= 1 && occupation.y <= 59)
		  {
			  //障碍物中心设置为1（以便于A*将障碍物所在位置识别为障碍物），第一圈的危险度在原来的基础上加0.0.8，
			  //第二圈的危险度在原来的基础上加0.8


			//  maze[occupation.x + 2][occupation.y + 2] = 31;
			  maze[occupation.x + 2][occupation.y + 1] = 31;
			  maze[occupation.x + 2][occupation.y] = 31;
			  maze[occupation.x + 2][occupation.y - 1] = 31;
			//  maze[occupation.x + 2][occupation.y - 2] = 31;

			  maze[occupation.x + 1][occupation.y + 2] = 31;
			  maze[occupation.x][occupation.y + 2] = 31;
			  maze[occupation.x - 1][occupation.y + 2] = 31;

			//  maze[occupation.x - 2][occupation.y + 2] = 31;
			  maze[occupation.x - 2][occupation.y + 1] = 31;
			  maze[occupation.x - 2][occupation.y] = 31;
			  maze[occupation.x - 2][occupation.y - 1] = 31;
			//  maze[occupation.x - 2][occupation.y - 2] = 31;

			  maze[occupation.x + 1][occupation.y - 2] = 31;
			  maze[occupation.x][occupation.y - 2] = 31;
			  maze[occupation.x - 1][occupation.y - 2] = 31;

			  maze[occupation.x + 1][occupation.y] = 32;
			  maze[occupation.x - 1][occupation.y] = 32;
			  maze[occupation.x][occupation.y + 1] = 32;
			  maze[occupation.x][occupation.y - 1] = 32;

			  maze[occupation.x + 1][occupation.y + 1] = 32;
			  maze[occupation.x - 1][occupation.y + 1] = 32;
			  maze[occupation.x + 1][occupation.y - 1] = 32;
			  maze[occupation.x - 1][occupation.y - 1] = 32;
			  //将障碍物移动方向上前面几格也设置为障碍物
			  if (i == 1)
			  {
				  // cout << "--------"<<'\n'<<"黄色1位于"<<'('<<occupation.x<<','<<occupation.y<<')'<<" 实际x_速度：" << Frame.robots_yellow(i).raw_vel_x() << " y_速度: " << Frame.robots_yellow(i).raw_vel_y() << endl;

				  float occu_vel_y = Frame.robots_yellow(i).raw_vel_x() / 100;   //数字地图：障碍物x方向全局速度
				  float occu_vel_x = -Frame.robots_yellow(i).raw_vel_y() / 100; //数字地图：障碍物y方向全局速度
				  float net_vel = sqrt(occu_vel_y*occu_vel_y + occu_vel_x * occu_vel_x);
				  //cout << "黄色1 数字x_速度：" << occu_vel_x << "黄色1 y_速度：" << occu_vel_y << "--net_vel: " << net_vel << endl;
				  //速度比例
				  float sum = abs(occu_vel_x) + abs(occu_vel_y);
				  float x_ration = abs(occu_vel_x) / sum;
				  float y_ration = abs(occu_vel_y) / sum;

				  int forward_num;  //在动态障碍物前进方向上设置forward_num个格为高危险度
				  double y_step, x_step;
				  if (y_ration > x_ration)
				  {

					  forward_num = net_vel / 2.5;
					  // cout << "yyyyyy_ration 前面" <<forward_num<<"格"<< endl;
					  y_step = occu_vel_y > 0 ? 1. : -1.;
					  x_step = (occu_vel_x > 0 ? 1. : -1.) * x_ration / y_ration;
				  }
				  if (y_ration < x_ration)
				  {
					  //cout << "xXXXx_ration 前面" << forward_num << "格" << endl;
					  forward_num = net_vel / 2.5;
					  x_step = occu_vel_x > 0 ? 1 : -1;
					  y_step = (occu_vel_y > 0 ? 1 : -1) * y_ration / x_ration;
				  }
				  if (x_ration == y_ration)
				  {
					  forward_num = net_vel / 2.5;
					  x_step = occu_vel_x > 0 ? 1 : -1;
					  y_step = occu_vel_y > 0 ? 1 : -1;
				  }
				  for (int i = 1; i < forward_num; i++)
				  {
					  // cout<< "前方"<<'('<<min(45, occupation.x + round(i * x_step))<<','<< min(60, occupation.y + round(i * y_step))<<") 被设置为: "<< forward_num + 1.3 - i<<endl;
					  maze[std::max(1, min(45, int(occupation.x + round(i * x_step))))][std::max(1, min(60, int(occupation.y + round(i * y_step))))] = forward_num + 1.3 - i;
				  }
			  }

			  

		  }
	  }

	  
	  for (int i = 0; i < bluenum; i++)
	  {
		  occupation = transp(Frame.robots_blue(i).x(), Frame.robots_blue(i).y());

		  if (Frame.robots_blue(i).robot_id() != CONTROLLED_CAR_ID) {
			  if (occupation.x >= 1 && occupation.x <= 44 && occupation.y >= 1 && occupation.y <= 59)
			  {
				  //occupation = transp(Frame.robots_blue(i).x(), Frame.robots_blue(i).y());

				  //障碍物中心设置为1（以便于A*将障碍物所在位置识别为障碍物），第一圈的危险度在原来的基础上加0.0.8，
			  //第二圈的危险度在原来的基础上加0.8

				//  maze[occupation.x + 2][occupation.y + 2] = 31;
				  maze[occupation.x + 2][occupation.y + 1] = 31;
				  maze[occupation.x + 2][occupation.y] = 31;
				  maze[occupation.x + 2][occupation.y - 1] = 31;
				//  maze[occupation.x + 2][occupation.y - 2] = 31;

				  maze[occupation.x + 1][occupation.y + 2] = 31;
				  maze[occupation.x][occupation.y + 2] = 31;
				  maze[occupation.x - 1][occupation.y + 2] = 31;

				//  maze[occupation.x - 2][occupation.y + 2] = 31;
				  maze[occupation.x - 2][occupation.y + 1] = 31;
				  maze[occupation.x - 2][occupation.y] = 31;
				  maze[occupation.x - 2][occupation.y - 1] = 31;
				//  maze[occupation.x - 2][occupation.y - 2] = 31;

				  maze[occupation.x + 1][occupation.y - 2] = 31;
				  maze[occupation.x][occupation.y - 2] = 31;
				  maze[occupation.x - 1][occupation.y - 2] = 31;

				  maze[occupation.x + 1][occupation.y] = 32;
				  maze[occupation.x - 1][occupation.y] = 32;
				  maze[occupation.x][occupation.y + 1] = 32;
				  maze[occupation.x][occupation.y - 1] = 32;

				  maze[occupation.x + 1][occupation.y + 1] = 32;
				  maze[occupation.x - 1][occupation.y + 1] = 32;
				  maze[occupation.x + 1][occupation.y - 1] = 32;
				  maze[occupation.x - 1][occupation.y - 1] = 32;


				  

				  //将障碍物移动方向上前面几格也设置为障碍物
				  float occu_vel_y = Frame.robots_yellow(i).raw_vel_x() / 100;   //数字地图：障碍物x方向全局速度
				  float occu_vel_x = -Frame.robots_yellow(i).raw_vel_y() / 100; //数字地图：障碍物y方向全局速度
				  float net_vel = sqrt(occu_vel_y*occu_vel_y + occu_vel_x * occu_vel_x);
				  //cout << "黄色1 x_速度：" << occu_vel_x << "黄色1 y_速度：" << occu_vel_y << "--net_vel: " << net_vel << endl;
				  //速度比例
				  float sum = abs(occu_vel_x) + abs(occu_vel_y);
				  float x_ration = abs(occu_vel_x) / sum;
				  float y_ration = abs(occu_vel_y) / sum;

				  int forward_num;  //在动态障碍物前进方向上设置forward_num个格为高危险度
				  double y_step, x_step;
				  if (y_ration > x_ration)
				  {
					  forward_num = net_vel / 2.5;
					  y_step = occu_vel_y > 0 ? 1 : -1;
					  x_step = (occu_vel_x > 0 ? 1 : -1) * x_ration / y_ration;
				  }
				  if (y_ration < x_ration)
				  {
					  forward_num = net_vel / 2.5;
					  x_step = occu_vel_x > 0 ? 1 : -1;
					  y_step = (occu_vel_y > 0 ? 1 : -1) * y_ration / x_ration;
				  }
				  if (x_ration == y_ration)
				  {
					  forward_num = net_vel / 2.5;
					  x_step = occu_vel_x > 0 ? 1 : -1;
					  y_step = occu_vel_y > 0 ? 1 : -1;
				  }
				  for (int i = 1; i < forward_num; i++)
				  {
					  maze[std::max(1, min(44, int(occupation.x + round(i * x_step))))][std::max(1, min(59, int(occupation.y + round(i * y_step))))] = forward_num + 1.3 - i;
				  }



				  /*static int count = 0;
				  if (count % 50 == 0)
				  {
					  for (int i = 0; i < 47; i++)
					  {
						  for (int j = 0; j < 62; j++)
						  {
							  cout << maze[i][j] << ' ';
						  }
						  cout << endl;
					  }

				  }
				  count++;*/
			  }
		  }


		  else if (Frame.robots_blue(i).robot_id() == CONTROLLED_CAR_ID) {
			  ID = i;
		  }
	  }

	  for (int i = 0; i < yellownum; i++)
	  {
		  occupation = transp(Frame.robots_yellow(i).x(), Frame.robots_yellow(i).y());  //障碍物坐标

		  if (occupation.x >= 1 && occupation.x <= 44 && occupation.y >= 1 && occupation.y <= 59)
		  {
			  maze[occupation.x][occupation.y] = 100;
		  }
	  }

	  for (int i = 0; i < bluenum; i++)
	  {
		  occupation = transp(Frame.robots_blue(i).x(), Frame.robots_blue(i).y());  //障碍物坐标
		  if (Frame.robots_blue(i).robot_id() != CONTROLLED_CAR_ID) {
			  if (occupation.x >= 1 && occupation.x <= 44 && occupation.y >= 1 && occupation.y <= 59)
			  {
				  maze[occupation.x][occupation.y] = 100;
			  }
		  }
	  }
	  //cout << "Update Map Success!" << endl;
	  return ID;

  }


  void Astar::Release(list<MyPoint*> path)
  {
	  for (list<MyPoint*>::iterator it = path.begin(); it != path.end();it++) 
	  {
		  delete *it;
	  }
  }
