#pragma once

#define yellow 0
#define blue 1

#include "basic.h"
#include "astar.h"

int GetFrame(Vision_DetectionFrame &frame, SOCKET sock, char* m_buf, int m_size, char* ip_buf, int ip_size, int flag, SOCKADDR_IN addr, int size);

void PrintRoboInfo(int id, int color, Vision_DetectionFrame Frame);

void PrintBallinfo(Vision_DetectionFrame Frame);

//void UpdateMap(vector<vector<int>>& maze, Vision_DetectionFrame Frame);