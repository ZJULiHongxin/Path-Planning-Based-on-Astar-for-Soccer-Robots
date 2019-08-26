/*----------------------------------------------------------------
// Copyright (C) 2019 李鸿鑫
// 版权所有。
//
// 文件名：main.cpp
// 文件功能描述：静态路径规划，使用A*
//
//
// 创建者：李鸿鑫(Hongxin Li)
// 时间：2019/8/26
//
// 版本：V1.0.0
//----------------------------------------------------------------*/

#include "astar.h"
#include "receive.h"
#include "send.h"
#include "wsadatainit.h"
#include "setupServer.h"
#include "mylocal_planner.h"
#include "zss_debug.pb.h"
#include "draw_map.h"



int SOCKADDR_IN_SIZE = sizeof(SOCKADDR_IN);

const int cmd_PORT = 50001;
const u_short vision_PORT = 23333;
const char* localIP = "127.0.0.1";
int send_BufLen = 10240;
char send_Buf[10240];
int debug_PORT = 20001;
const size_t IP_BUF_SIZE = 256;
char ip_buf[IP_BUF_SIZE];
char cmd_buffer[10240];
int cmd_buffer_size = 10240;
const size_t MSG_BUF_SIZE = 2048;
char msg_buf[MSG_BUF_SIZE];
int dt = 10;
int flag = 0;
int ID;

int main() {

	
		WSADATA wsa_data;
		SOCKET sock_serv = INVALID_SOCKET;
		SOCKADDR_IN addr_serv, addr_clt;
		SOCKADDR_IN addr_recv;
		SOCKADDR_IN addr_cmd;   //运动指令
		SOCKADDR_IN addr_debug; //绘制路径指令


		char ip_buf[IP_BUF_SIZE];
		char msg_buf[MSG_BUF_SIZE];
		int ret_val = 0;
		//
		ret_val = WSAStartup(MAKEWORD(2, 2), &wsa_data);
		if (ret_val != 0) {
			cerr << "WSAStartup() function failed with error: " << WSAGetLastError() << "\n";
			system("pause");
			return 1;
		}
		//
		//SecureZeroMemory(&addr_serv, SOCKADDR_IN_SIZE);
		addr_serv.sin_family = AF_INET;
		addr_serv.sin_port = htons(vision_PORT);
		addr_serv.sin_addr.S_un.S_addr = ADDR_ANY;
		addr_debug.sin_family = AF_INET;
		addr_debug.sin_port = htons(debug_PORT);
		addr_debug.sin_addr.s_addr = inet_addr("127.0.0.1");
		//创建Socket对象
		SOCKET SendSocket;
		SendSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
		//设置服务器地址
		addr_recv.sin_family = AF_INET;
		addr_recv.sin_port = htons(cmd_PORT);
		addr_recv.sin_addr.s_addr = inet_addr("127.0.0.1");
		//向服务器发送数据报
		printf("Sending a datagram to the receiver...\n");
		//

		sock_serv = socket(addr_serv.sin_family, SOCK_DGRAM, IPPROTO_UDP);
		if (sock_serv == INVALID_SOCKET) {
			cerr << "socket() function failed with error: " << WSAGetLastError() << "\n";
			WSACleanup();
			system("pause");
			return 1;
		}
		//
		ret_val = bind(sock_serv, (SOCKADDR*)& addr_serv, SOCKADDR_IN_SIZE);
		if (ret_val != 0) {
			cerr << "bind() function failed with error: " << WSAGetLastError() << "\n";
			system("pause");
			return 1;
		}
		cout << "A UDP server has started successfully..." << endl;
	/*初始化结束*/

	/*初始化一个Astar对象，用来进行路径规划*/
	Astar astar;

	/*摄像机传来的当前帧*/
	Vision_DetectionFrame Frame;

	/* 离散化的数字地图，用来近似表征实际环境，分辨率是44×59。每一格用0~100之间的浮点数表示此处的危险度，100表示绝对障碍物，
	 * 0表示绝对安全
	 */
	astar.maze = {
		//0                   1                     2                     3米                                                                       6
		{1,1,1,1,1,1,1,1,1,1,1,  1,1,1,1,1,1,1,1,1,1,  1,1,1,1,1,1,1,1,1,1,  1,1,1,1,1,1,1,1,1,1,  1,1,1,1,1,1,1,1,1,1,  1,1,1,1,1,1,1,1,1,1  },
		{1,0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,1  },
		{1,0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,1  },
		{1,0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,1  },
		{1,0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,1  },
		{1,0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,1  },

		{1,0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,1  },
		{1,0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,1  },
		{1,0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,1  },
		{1,0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,1  },
		{1,0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,1  }, //1

		{1,0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,1  },
		{1,0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,1  },
		{1,0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,1  },
		{1,0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,1  },
		{1,0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,1  },

		{1,0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,1  },
		{1,0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,1  },
		{1,0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,1  },
		{1,0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,1  },
		{1,0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,1  },//2

		{1,0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,1  },
		{1,0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,1  },
		{1,0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,1  },
		{1,0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,1  },
		{1,0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,1  },

		{1,0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,1  },
		{1,0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,1  },
		{1,0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,1  },
		{1,0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,1  },
		{1,0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,1  }, //3

		{1,0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,1  },
		{1,0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,1  },
		{1,0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,1  },
		{1,0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,1  },
		{1,0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,1  },

		{1,0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,1  },
		{1,0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,1  },
		{1,0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,1  },
		{1,0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,1  },
		{1,0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,1  }, //4


		{1,0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,1  },
		{1,0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,1  },
		{1,0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,1  },
		{1,0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,1  },
		{1,1,1,1,1,1,1,1,1,1,1,  1,1,1,1,1,1,1,1,1,1,  1,1,1,1,1,1,1,1,1,1,  1,1,1,1,1,1,1,1,1,1,  1,1,1,1,1,1,1,1,1,1,  1,1,1,1,1,1,1,1,1,1  },  //4.5米
	};

	/*初始化危险度阈值*/
	astar.th = 31;

	/*设定目标点*/
	MyPoint tar_p = transp(2500, -1500);
	MyPoint tttar_p = tar_p;

	SecureZeroMemory(msg_buf, MSG_BUF_SIZE);

	//list<MyPoint> final_path;
	//MyPoint one(1, 1), two(10, 20), three(20, 45), four(35, 10), five(1, 1);
	//final_path.push_back(one); final_path.push_back(two); final_path.push_back(three); final_path.push_back(four); final_path.push_back(five);

	////下面开始进行第一次A-star寻路，根据当前坐标cur_p和目标坐标tar_p。获得一个序列final_path;
	
	//获得当前信息，存储在Frame中；ret_val是状态值用于检错。
	ret_val = GetFrame(Frame, sock_serv, msg_buf, MSG_BUF_SIZE, ip_buf, IP_BUF_SIZE, 0, addr_clt, SOCKADDR_IN_SIZE);
	ID = astar.UpdateMap(Frame);

	MyPoint cur_p = transp(Frame.robots_blue(ID).x(), Frame.robots_blue(ID).y());
	MyPoint ssstart_p = transp(-2500, 1500);	//MyPoint(10, 10)
	VelPoint real_t_vp;
	double cur_ori;


	while (!(abs(cur_p.x - ssstart_p.x) < 1 && abs(cur_p.y - ssstart_p.y) < 1))
	{
		ret_val = GetFrame(Frame, sock_serv, msg_buf, MSG_BUF_SIZE, ip_buf, IP_BUF_SIZE, 0, addr_clt, SOCKADDR_IN_SIZE);

		cur_p = transp(Frame.robots_blue(ID).x(), Frame.robots_blue(ID).y());
		cur_ori = Frame.robots_blue(ID).orientation();
		real_t_vp = GetTarVel(cur_p, ssstart_p, cur_ori);
		SendCommand(SendSocket, cmd_buffer, addr_recv, CONTROLLED_CAR_ID, 0, real_t_vp.vx, real_t_vp.vy);
	}

	GetFrame(Frame, sock_serv, msg_buf, MSG_BUF_SIZE, ip_buf, IP_BUF_SIZE, 0, addr_clt, SOCKADDR_IN_SIZE);
	astar.UpdateMap(Frame);

	//从起点往终点搜索一条路径
	list<MyPoint*> path_from_start = astar.GetPath(cur_p, tar_p, false);
	list<MyPoint> final_path_from_start = astar.ShortCutting(path_from_start);
	DrawMap(SendSocket, send_Buf, addr_debug, final_path_from_start);
	astar.InitAstar();
	//从终点往起点搜索一条路径
	list<MyPoint *> path_from_end = astar.GetPath(cur_p, tar_p, false);
	list<MyPoint> final_path_from_end = astar.ShortCutting(path_from_end);
	DrawMap(SendSocket, send_Buf, addr_debug, final_path_from_end);
	//最终路径选择两套路径中拐点最少的一条
	list<MyPoint> final_path;
	final_path_from_end.size() > final_path_from_start.size() ? final_path = final_path_from_start : final_path = final_path_from_end;

	//绘制路径
	//DrawMap(SendSocket, send_Buf, addr_debug, final_path);
	
	MyPoint np;

	bool forward = 1;

	//大循环
	while (ret_val >= 0) 
	{
		if (abs(cur_p.x - tar_p.x) < 1 && abs(cur_p.y - tar_p.y) < 1)
		{
			
			if (forward == 1)
				tar_p = ssstart_p;
			else
				tar_p = tttar_p;

			forward = !forward;

			GetFrame(Frame, sock_serv, msg_buf, MSG_BUF_SIZE, ip_buf, IP_BUF_SIZE, 0, addr_clt, SOCKADDR_IN_SIZE);
			astar.UpdateMap(Frame);

			//从起点往终点搜索一条路径
			astar.InitAstar();
			path_from_start.clear();
			final_path_from_start.clear();
			path_from_start = astar.GetPath(cur_p, tar_p, false);
			final_path_from_start = astar.ShortCutting(path_from_start);

			astar.InitAstar();
			//从终点往起点搜索一条路径
			path_from_end.clear();
			final_path_from_end.clear();
			path_from_end = astar.GetPath(cur_p, tar_p, false);
			final_path_from_end = astar.ShortCutting(path_from_end);

			//最终路径选择两套路径中拐点最少的一条
			final_path.clear();
			final_path_from_end.size() > final_path_from_start.size() ? final_path = final_path_from_start : final_path = final_path_from_end;
			
			//绘制路径
			//DrawMap(SendSocket, send_Buf, addr_debug, path_from_start);
			DrawMap(SendSocket, send_Buf, addr_debug, final_path);

		}

		final_path.pop_front();
		np = *(final_path.begin());

		

		while (!(abs(cur_p.x - np.x) < 1 && abs(cur_p.y - np.y) < 1)) {
			//获得当前信息，存储在Frame中；ret_val是状态值用于检错。
			GetFrame(Frame, sock_serv, msg_buf, MSG_BUF_SIZE, ip_buf, IP_BUF_SIZE, 0, addr_clt, SOCKADDR_IN_SIZE);
			astar.UpdateMap(Frame);

			real_t_vp = GetTarVel(cur_p, np, cur_ori);
			SendCommand(SendSocket, cmd_buffer, addr_recv, CONTROLLED_CAR_ID, 0, real_t_vp.vx, real_t_vp.vy);
			
			//cur_p 为当前位置，转换为数字地图坐标；
			cur_p = transp(Frame.robots_blue(ID).x(), Frame.robots_blue(ID).y());
			cur_ori = Frame.robots_blue(ID).orientation();
			int tempx[2] = { cur_p.x,np.x };
			int tempy[2] = { cur_p.y,np.y };
			
		}
	}
	if (ret_val == 0) {
		cout << "connection is closed..." << endl;
	}
	else {
		cerr << "recvfrom() function failed with error: " << WSAGetLastError() << "\n";
		closesocket(sock_serv);
		WSACleanup();
		system("pause");
		return 1;
	}



	ret_val = shutdown(sock_serv, SD_BOTH);
	if (ret_val == SOCKET_ERROR) {
		cerr << "shutdown() function failed with error: " << WSAGetLastError() << "\n";
		closesocket(sock_serv);
		WSACleanup();
		system("pause");
		return 1;
	}
	closesocket(sock_serv);
	closesocket(SendSocket);

	WSACleanup();
	cout << "server shutdown..." << endl;
	system("pause");
	return 0;
}
