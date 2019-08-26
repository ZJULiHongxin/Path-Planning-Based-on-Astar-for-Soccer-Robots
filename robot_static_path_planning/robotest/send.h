#pragma once

#include "basic.h"

void SendCommand(SOCKET SendSocket, char* cmd_buffer, SOCKADDR_IN addr_recv, int id, int vr, int vx, int vy, bool is_kick=false, int power=100, int spin=0);