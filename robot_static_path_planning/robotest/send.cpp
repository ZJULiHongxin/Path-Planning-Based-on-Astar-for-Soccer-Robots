/*----------------------------------------------------------------
// Copyright (C) 2019 李鸿鑫
// 版权所有。
//
// 文件名：mylocal_planner.cpp
// 文件功能描述：给场上受自己控制的机器人发送速度信息
//
//
// 创建者：李鸿鑫(Hongxin Li)
// 时间：2019/8/26
//
// 版本：V1.0.0
//----------------------------------------------------------------*/

#include "basic.h"
#include "send.h"

void SendCommand(SOCKET SendSocket, char* cmd_buffer, SOCKADDR_IN addr_recv, int id, int vr, int vx, int vy, bool is_kick, int power, int spin) {
	Robots_Command cmd;
	Robot_Command* p_cmd;

	p_cmd = cmd.add_command();
	p_cmd->set_robot_id(id);
	p_cmd->set_velocity_r(vr);
	p_cmd->set_velocity_x(vx);
	p_cmd->set_velocity_y(vy);
	p_cmd->set_kick(is_kick);
	p_cmd->set_power(power);
	p_cmd->set_dribbler_spin(spin);

	int cmd_size = cmd.command_size();
	extern int cmd_buffer_size;
	extern int send_BufLen;
	cmd.SerializeToArray(cmd_buffer, cmd_buffer_size);
	sendto(SendSocket, cmd_buffer, send_BufLen, 0, (SOCKADDR*)& addr_recv, sizeof(addr_recv));
}