/*----------------------------------------------------------------
// Copyright (C) 2019 �����
// ��Ȩ���С�
//
// �ļ�����mylocal_planner.cpp
// �ļ��������������úñ�������������������
//
//
// �����ߣ������(Hongxin Li)
// ʱ�䣺2019/8/26
//
// �汾��V1.0.0
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