#include "wsadatainit.h"

WSADATA initWSA(void) {
	WSADATA wsa_data;
	int ret_val = 0;
	ret_val = WSAStartup(MAKEWORD(2, 2), &wsa_data);
	if (ret_val != 0) {
		cerr << "WSAStartup() function failed with error: " << WSAGetLastError() << "\n";
		system("pause");
	}
	return wsa_data;
}
