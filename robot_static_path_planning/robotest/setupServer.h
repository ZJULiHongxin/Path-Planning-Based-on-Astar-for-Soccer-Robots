#pragma once
#include "basic.h"

void setupServer(SOCKADDR_IN server, int port);
void setupReceiver(SOCKADDR_IN server, const char* localip);