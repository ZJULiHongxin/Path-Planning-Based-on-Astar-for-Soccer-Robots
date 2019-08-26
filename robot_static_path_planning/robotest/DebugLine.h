#pragma once
#include "basic.h"
#include "astar.h"

void GetKeypoint(int* x, int* y, list<MyPoint> path);

void DrawDebugLine(int* x, int* y, int len, char* send_Buf, int send_BufLen, SOCKET SendSocket, SOCKADDR_IN addr_debug);


