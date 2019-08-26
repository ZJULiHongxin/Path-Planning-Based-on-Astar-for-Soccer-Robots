#include "DebugLine.h"

void GetKeypoint(int* x, int* y, list<MyPoint> path) {
	int i = 0;
	list<MyPoint> ::iterator iter;
	for (iter = path.begin(); iter != path.end(); iter++)
	{
		x[i] = inv_x(*iter);
		y[i] = inv_y(*iter);
//		cout << "temp" << '(' << x[i] << ',' << y[i] << ')' << endl;
		i++;
	}
}

void DrawDebugLine(int* x, int* y, int len, char* send_Buf,int send_BufLen,SOCKET SendSocket, SOCKADDR_IN addr_debug) {

	Debug_Msgs msg;
	for (int j = 1; j < len; j++)
	{
		Debug_Msg* p_msg;
		Debug_Line* aline = new Debug_Line;
		Point* start = new Point;
		Point* end = new Point;

		p_msg = msg.add_msgs();
		p_msg->set_type(Debug_Msg_Debug_Type_LINE);
		p_msg->set_color(Debug_Msg_Color_RED);

		aline->set_allocated_start(start);
		aline->set_allocated_end(end);

		p_msg->set_allocated_line(aline);

		cout << '(' << x[j - 1] << ',' << -y[j - 1] << ')' << "to" << '(' << x[j] << ',' << -y[j] << ')' << endl;
		start->set_x(x[j - 1]);
		start->set_y(-y[j - 1]);
		end->set_x(x[j]);
		end->set_y(-y[j]);
		aline->set_forward(1);
		aline->set_back(0);
		msg.SerializeToArray(send_Buf, send_BufLen);
		cout << sendto(SendSocket, send_Buf, send_BufLen, 0, (SOCKADDR*)& addr_debug, sizeof(addr_debug)) << endl;
	}
}