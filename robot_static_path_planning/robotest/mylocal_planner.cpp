/*----------------------------------------------------------------
// Copyright (C) 2019 �����
// ��Ȩ���С�
//
// �ļ�����mylocal_planner.cpp
// �ļ��������������ݻ���������һ��Ŀ���ľ�����㵱ǰ�ٶȣ�ȷ��������������·����ʻ
//
//
// �����ߣ������(Hongxin Li)
// ʱ�䣺2019/8/26
//
// �汾��V1.0.0
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
		t_glb_vx = 30 * dx;//Ŀ��ȫ��x�ٶ�
		t_glb_vy = 30 * dy;
	}
	else {
		t_glb_vx = SPEED * cos(alpha);//Ŀ��ȫ��x�ٶ�
		t_glb_vy = SPEED * sin(alpha);
	}

	double t_vx = t_glb_vy * sin(ori) + t_glb_vx * cos(ori);//Ŀ������x�ٶ�
	double t_vy = t_glb_vy * cos(ori) - t_glb_vx * sin(ori);

	temp.vx = t_vy;
	temp.vy = t_vx;

	//cout << "�����˵�ǰ�Ƕ�:" << ori << endl;
	//cout << "���ֵ�ͼȫ��Ŀ���ٶȣ�t_glb_vx:" << t_glb_vx << ",t_glb_vy:" << t_glb_vy << endl;
	//cout << "���ֵ�ͼ����Ŀ���ٶȣ�t_vx:" << t_vx << ",t_vy:" << t_vy << endl;
	//cout << "ԭʼ��ͼ����Ŀ���ٶȣ�real_t_vx:" << temp.vx << ",real_t_vy:" << temp.vy << endl;

	return temp;
}