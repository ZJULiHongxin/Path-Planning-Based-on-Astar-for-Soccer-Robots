/*----------------------------------------------------------------
// Copyright (C) 2019 李鸿鑫
// 版权所有。
//
// 文件名：receive.cpp
// 文件功能描述：接受当前帧中场上所有机器人的信息
//
//
// 创建者：李鸿鑫(Hongxin Li)
// 时间：2019/8/26
//
// 版本：V1.0.0
//----------------------------------------------------------------*/

#include "receive.h"

int GetFrame(Vision_DetectionFrame &frame,SOCKET sock, char* m_buf, int m_size ,char* ip_buf, int ip_size,int flag,SOCKADDR_IN addr, int size) {
	int ret_val = recvfrom(sock, m_buf, m_size, 0, (SOCKADDR*)& addr, &size);
	inet_ntop(addr.sin_family, &addr, ip_buf, ip_size);
	frame.ParseFromArray(m_buf, m_size);
	return ret_val;
}


void PrintRoboInfo(int id, int color, Vision_DetectionFrame Frame) {
	int i = id;
	if (color == yellow) {
		cout << "robot_yellow (" << i << ") :" << endl;
		cout << "valid : " << Frame.robots_yellow(i).valid() << endl;
		cout << "robot_id : " << Frame.robots_yellow(i).robot_id() << endl;
		cout << "x : " << Frame.robots_yellow(i).x() << endl;
		cout << "y : " << Frame.robots_yellow(i).y() << endl;
		cout << "orientation : " << Frame.robots_yellow(i).orientation() << endl;
		cout << "vel_x : " << Frame.robots_yellow(i).vel_x() << endl;
		cout << "vel_y : " << Frame.robots_yellow(i).vel_y() << endl;
		cout << "rotate_vel : " << Frame.robots_yellow(i).rotate_vel() << endl;
		cout << "accelerate_x : " << Frame.robots_yellow(i).accelerate_x() << endl;
		cout << "accelerate_y : " << Frame.robots_yellow(i).accelerate_y() << endl;
		cout << "raw_x : " << Frame.robots_yellow(i).raw_x() << endl;
		cout << "raw_y : " << Frame.robots_yellow(i).raw_y() << endl;
		cout << "raw_orientation : " << Frame.robots_yellow(i).raw_orientation() << endl;
		cout << "raw_vel_x : " << Frame.robots_yellow(i).raw_vel_x() << endl;
		cout << "raw_vel_y : " << Frame.robots_yellow(i).raw_vel_y() << endl;
		cout << "raw_rotate_vel : " << Frame.robots_yellow(i).raw_rotate_vel() << endl;
		cout << endl;
	}
	else if (color == blue) {
		cout << "robot_blue (" << i << ") :" << endl;
		cout << "valid : " << Frame.robots_blue(i).valid() << endl;
		cout << "robot_id : " << Frame.robots_blue(i).robot_id() << endl;
		cout << "x : " << Frame.robots_blue(i).x() << endl;
		cout << "y : " << Frame.robots_blue(i).y() << endl;
		cout << "orientation : " << Frame.robots_blue(i).orientation() << endl;
		cout << "vel_x : " << Frame.robots_blue(i).vel_x() << endl;
		cout << "vel_y : " << Frame.robots_blue(i).vel_y() << endl;
		cout << "rotate_vel : " << Frame.robots_blue(i).rotate_vel() << endl;
		cout << "accelerate_x : " << Frame.robots_blue(i).accelerate_x() << endl;
		cout << "accelerate_y : " << Frame.robots_blue(i).accelerate_y() << endl;
		cout << "raw_x : " << Frame.robots_blue(i).raw_x() << endl;
		cout << "raw_y : " << Frame.robots_blue(i).raw_y() << endl;
		cout << "raw_orientation : " << Frame.robots_blue(i).raw_orientation() << endl;
		cout << "raw_vel_x : " << Frame.robots_blue(i).raw_vel_x() << endl;
		cout << "raw_vel_y : " << Frame.robots_blue(i).raw_vel_y() << endl;
		cout << "raw_rotate_vel : " << Frame.robots_blue(i).raw_rotate_vel() << endl;
		cout << endl;
	}
}

void PrintBallinfo(Vision_DetectionFrame Frame) {
	cout << "balls' vel_x : " << Frame.balls().vel_x() << endl;
	cout << "balls' vel_y : " << Frame.balls().vel_y() << endl;
	cout << "balls' area : " << Frame.balls().area() << endl;
	cout << "balls' x : " << Frame.balls().x() << endl;
	cout << "balls' y : " << Frame.balls().y() << endl;
	cout << "balls' height : " << Frame.balls().height() << endl;
	cout << "balls' ball_state : " << Frame.balls().ball_state() << endl;
	cout << "balls' last_touch : " << Frame.balls().last_touch() << endl;
	cout << "balls' valid : " << Frame.balls().valid() << endl;
	cout << "balls' raw_x : " << Frame.balls().raw_x() << endl;
	cout << "balls' raw_y : " << Frame.balls().raw_y() << endl;
	cout << "balls' chip_predict_x : " << Frame.balls().chip_predict_x() << endl;
	cout << "balls' chip_predict_y : " << Frame.balls().chip_predict_y() << endl;
	cout << endl;
}


