#include "basic.h"
#include "draw_map.h"
#include "astar.h"

extern int send_BufLen;

//extern list<MyPoint> final_path;

//画最终路径
//void DrawMap(SOCKET SendSocket, char* send_Buf, SOCKADDR_IN addr_debug, list<MyPoint> final_path) {
//	//计算路径
//	list<MyPoint> ::iterator iter;
//	//temp_x和temp_y存储路径点
//	int* temp_x = new int[final_path.size()];
//	int* temp_y = new int[final_path.size()];
//	int i = 0;
//	for (iter = final_path.begin(); iter != final_path.end(); iter++)
//	{
//		temp_x[i] = inv_x(*iter);
//		temp_y[i] = inv_y(*iter);
//		cout << "temp" << '(' << temp_x[i] << ',' << temp_y[i] << ')' << endl;
//		i++;
//
//	}
//
//	Debug_Msgs msg;
//
//	for (int j = 1; j < i; j++)
//	{
//
//		Debug_Msg* p_msg;
//		Debug_Line* aline = new Debug_Line;
//		Point* start = new Point;
//		Point* end = new Point;
//
//		p_msg = msg.add_msgs();
//		p_msg->set_type(Debug_Msg_Debug_Type_LINE);
//		p_msg->set_color(Debug_Msg_Color_RED);
//
//		aline->set_allocated_start(start);
//		aline->set_allocated_end(end);
//
//		p_msg->set_allocated_line(aline);
//
//		cout << '(' << temp_x[j - 1] << ',' << temp_y[j - 1] << ')' << "to" << '(' << temp_x[j] << ',' << temp_y[j] << ')' << endl;
//		start->set_x(temp_x[j - 1]);
//		start->set_y(temp_y[j - 1]);
//		end->set_x(temp_x[j]);
//		end->set_y(temp_y[j]);
//		aline->set_forward(1);
//		aline->set_back(0);
//		msg.SerializeToArray(send_Buf, send_BufLen);
//		cout << sendto(SendSocket, send_Buf, send_BufLen, 0, (SOCKADDR*)& addr_debug, sizeof(addr_debug)) << endl;
//	}
//}

void DrawMap(SOCKET SendSocket, char* send_Buf, SOCKADDR_IN addr_debug, list<MyPoint> final_path) {
	//计算路径
	Debug_Msgs msg;

	for (list<MyPoint> ::iterator iter = final_path.begin(); iter != --final_path.end() ;)
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

		start->set_x(inv_x(*iter));
		start->set_y(inv_y(*(iter++)));
		end->set_x(inv_x(*iter));
		end->set_y(inv_y(*(iter)));
		//cout << "路径 " << '(' << start->x() << ',' << start->y() << ')' << " to " << '(' << end->x() << ',' << end->y() << ')' << endl;

		aline->set_forward(1);
		aline->set_back(0);
		msg.SerializeToArray(send_Buf, send_BufLen);
		sendto(SendSocket, send_Buf, send_BufLen, 0, (SOCKADDR*)& addr_debug, sizeof(addr_debug));
	}
}

//画原始路径
//void DrawMap(SOCKET SendSocket, char* send_Buf, SOCKADDR_IN addr_debug, list<MyPoint*> path) {
//	//计算路径
//	list<MyPoint*> ::iterator iter;
//	//temp_x和temp_y存储路径点
//	int* temp_x = new int[path.size()];
//	int* temp_y = new int[path.size()];
//	int i = 0;
//	for (iter = path.begin(); iter != path.end(); iter++)
//	{
//		
//		temp_x[i] = inv_x(*(*iter));
//		temp_y[i] = inv_y(*(*iter));
//		//cout << "temp" << '(' << temp_x[i] << ',' << temp_y[i] << ')' << endl;
//		i++;
//	}
//
//	Debug_Msgs msg;
//
//	for (int j = 0; j < i; j++)
//	{
//
//		Debug_Msg* p_msg;
//		Debug_Line* aline = new Debug_Line;
//		Point* start = new Point;
//		Point* end = new Point;
//
//		p_msg = msg.add_msgs();
//		p_msg->set_type(Debug_Msg_Debug_Type_LINE);
//		p_msg->set_color(Debug_Msg_Color_BLUE);
//
//		aline->set_allocated_start(start);
//		aline->set_allocated_end(end);
//
//		p_msg->set_allocated_line(aline);
//
//		cout << '(' << temp_x[j - 1] << ',' << temp_y[j - 1] << ')' << "to" << '(' << temp_x[j] << ',' << temp_y[j] << ')' << endl;
//		start->set_x(temp_x[j - 1]);
//		start->set_y(temp_y[j - 1]);
//		end->set_x(temp_x[j]);
//		end->set_y(temp_y[j]);
//		aline->set_forward(1);
//		aline->set_back(0);
//		msg.SerializeToArray(send_Buf, send_BufLen);
//		cout << sendto(SendSocket, send_Buf, send_BufLen, 0, (SOCKADDR*)& addr_debug, sizeof(addr_debug)) << endl;
//	}
//}

void DrawMap(SOCKET SendSocket, char* send_Buf, SOCKADDR_IN addr_debug, list<MyPoint*> path) {
	//计算路径
	Debug_Msgs msg;

	for (list<MyPoint*> ::iterator iter = path.begin(); iter != --path.end(); )
	{

		Debug_Msg* p_msg;
		Debug_Line* aline = new Debug_Line;
		Point* start = new Point;
		Point* end = new Point;

		p_msg = msg.add_msgs();
		p_msg->set_type(Debug_Msg_Debug_Type_LINE);
		p_msg->set_color(Debug_Msg_Color_BLUE);

		aline->set_allocated_start(start);
		aline->set_allocated_end(end);

		p_msg->set_allocated_line(aline);

		//cout << "路径 "<<'(' << temp_x[j - 1] << ',' << temp_y[j - 1] << ')' << " to " << '(' << temp_x[j] << ',' << temp_y[j] << ')' << endl;
		start->set_x(inv_x(*(*iter)));
		start->set_y(inv_y(*(*(iter++))));
		end->set_x(inv_x(*(*iter)));
		end->set_y(inv_y(*(*(iter))));
		aline->set_forward(1);
		aline->set_back(0);
		msg.SerializeToArray(send_Buf, send_BufLen);
		sendto(SendSocket, send_Buf, send_BufLen, 0, (SOCKADDR*)& addr_debug, sizeof(addr_debug));
	}
}