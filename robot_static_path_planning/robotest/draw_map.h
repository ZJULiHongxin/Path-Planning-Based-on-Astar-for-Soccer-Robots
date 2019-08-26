#pragma once

#include "basic.h"

void DrawMap(SOCKET SendSocket, char* send_Buf, SOCKADDR_IN addr_debug, list<MyPoint> final_path);
void DrawMap(SOCKET SendSocket, char* send_Buf, SOCKADDR_IN addr_debug, list<MyPoint*> path);

