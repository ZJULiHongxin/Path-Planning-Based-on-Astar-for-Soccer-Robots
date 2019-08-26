/*----------------------------------------------------------------
// Copyright (C) 2019 李鸿鑫
// 版权所有。
//
// 文件名：mylocal_planner.cpp
// 文件功能描述：配置好本程序与仿真软件的链接
//
//
// 创建者：李鸿鑫(Hongxin Li)
// 时间：2019/8/26
//
// 版本：V1.0.0
//----------------------------------------------------------------*/

#include "setupServer.h"

void setupServer(SOCKADDR_IN server, int port) {
	server.sin_family = AF_INET;
	server.sin_port = htons(port);
	server.sin_addr.S_un.S_addr = ADDR_ANY;
}

void setupReceiver(SOCKADDR_IN server, const char* localip) {

	server.sin_family = AF_INET;
	server.sin_port = htons(50001);
	server.sin_addr.s_addr = inet_addr(localip);
}