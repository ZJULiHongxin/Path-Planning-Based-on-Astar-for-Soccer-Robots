/*----------------------------------------------------------------
// Copyright (C) 2019 李鸿鑫
// 版权所有。
//
// 文件名：mylocal_planner.cpp
// 文件功能描述：根据机器人与下一个目标点的距离计算当前速度，确保机器人沿期望路径行驶
//
//
// 创建者：李鸿鑫(Hongxin Li)
// 时间：2019/8/26
//
// 版本：V1.0.0
//----------------------------------------------------------------*/


#include "mylocal_planner.h"

double GetDistance(MyPoint cur_p, MyPoint np) {
	return sqrt((cur_p.x - np.x)*(cur_p.x - np.x) + (cur_p.y - np.y)*(cur_p.y - np.y));
}

VelPoint GetTarVel(MyPoint cur_p, MyPoint np, double ori) {
	VelPoint temp;
	const int SPEED = 300;

	double dx = np.x - cur_p.x;
	double dy = np.y - cur_p.y;
	double alpha = atan2(dy, dx);
	//cout << "alpha:" << alpha << endl;

	double t_glb_vx;
	double t_glb_vy;

	double dis = GetDistance(cur_p, np);

	if (dis <= 10) {
		t_glb_vx = 30 * dx;//目标全局x速度
		t_glb_vy = 30 * dy;
	}
	else {
		t_glb_vx = SPEED * cos(alpha);//目标全局x速度
		t_glb_vy = SPEED * sin(alpha);
	}

	double t_vx = t_glb_vy * sin(ori) + t_glb_vx * cos(ori);//目标自身x速度
	double t_vy = t_glb_vy * cos(ori) - t_glb_vx * sin(ori);

	temp.vx = t_vy;
	temp.vy = t_vx;

	//cout << "机器人当前角度:" << ori << endl;
	//cout << "数字地图全局目标速度：t_glb_vx:" << t_glb_vx << ",t_glb_vy:" << t_glb_vy << endl;
	//cout << "数字地图本地目标速度：t_vx:" << t_vx << ",t_vy:" << t_vy << endl;
	//cout << "原始地图本地目标速度：real_t_vx:" << temp.vx << ",real_t_vy:" << temp.vy << endl;

	return temp;
}