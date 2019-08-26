/*----------------------------------------------------------------
// Copyright (C) 2019 �����
// ��Ȩ���С�
//
// �ļ�����main.cpp
// �ļ�������������̬·���滮��ʹ��A*
//
//
// �����ߣ������(Hongxin Li)
// ʱ�䣺2019/8/26
//
// �汾��V1.0.0
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
		SOCKADDR_IN addr_cmd;   //�˶�ָ��
		SOCKADDR_IN addr_debug; //����·��ָ��


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
		//����Socket����
		SOCKET SendSocket;
		SendSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
		//���÷�������ַ
		addr_recv.sin_family = AF_INET;
		addr_recv.sin_port = htons(cmd_PORT);
		addr_recv.sin_addr.s_addr = inet_addr("127.0.0.1");
		//��������������ݱ�
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
	/*��ʼ������*/

	/*��ʼ��һ��Astar������������·���滮*/
	Astar astar;

	/*����������ĵ�ǰ֡*/
	Vision_DetectionFrame Frame;

	/* ��ɢ�������ֵ�ͼ���������Ʊ���ʵ�ʻ������ֱ�����44��59��ÿһ����0~100֮��ĸ�������ʾ�˴���Σ�նȣ�100��ʾ�����ϰ��
	 * 0��ʾ���԰�ȫ
	 */
	astar.maze = {
		//0                   1                     2                     3��                                                                       6
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
		{1,1,1,1,1,1,1,1,1,1,1,  1,1,1,1,1,1,1,1,1,1,  1,1,1,1,1,1,1,1,1,1,  1,1,1,1,1,1,1,1,1,1,  1,1,1,1,1,1,1,1,1,1,  1,1,1,1,1,1,1,1,1,1  },  //4.5��
	};

	/*��ʼ��Σ�ն���ֵ*/
	astar.th = 31;

	/*�趨Ŀ���*/
	MyPoint tar_p = transp(2500, -1500);
	MyPoint tttar_p = tar_p;

	SecureZeroMemory(msg_buf, MSG_BUF_SIZE);

	//list<MyPoint> final_path;
	//MyPoint one(1, 1), two(10, 20), three(20, 45), four(35, 10), five(1, 1);
	//final_path.push_back(one); final_path.push_back(two); final_path.push_back(three); final_path.push_back(four); final_path.push_back(five);

	////���濪ʼ���е�һ��A-starѰ·�����ݵ�ǰ����cur_p��Ŀ������tar_p�����һ������final_path;
	
	//��õ�ǰ��Ϣ���洢��Frame�У�ret_val��״ֵ̬���ڼ��
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

	//��������յ�����һ��·��
	list<MyPoint*> path_from_start = astar.GetPath(cur_p, tar_p, false);
	list<MyPoint> final_path_from_start = astar.ShortCutting(path_from_start);
	DrawMap(SendSocket, send_Buf, addr_debug, final_path_from_start);
	astar.InitAstar();
	//���յ����������һ��·��
	list<MyPoint *> path_from_end = astar.GetPath(cur_p, tar_p, false);
	list<MyPoint> final_path_from_end = astar.ShortCutting(path_from_end);
	DrawMap(SendSocket, send_Buf, addr_debug, final_path_from_end);
	//����·��ѡ������·���йյ����ٵ�һ��
	list<MyPoint> final_path;
	final_path_from_end.size() > final_path_from_start.size() ? final_path = final_path_from_start : final_path = final_path_from_end;

	//����·��
	//DrawMap(SendSocket, send_Buf, addr_debug, final_path);
	
	MyPoint np;

	bool forward = 1;

	//��ѭ��
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

			//��������յ�����һ��·��
			astar.InitAstar();
			path_from_start.clear();
			final_path_from_start.clear();
			path_from_start = astar.GetPath(cur_p, tar_p, false);
			final_path_from_start = astar.ShortCutting(path_from_start);

			astar.InitAstar();
			//���յ����������һ��·��
			path_from_end.clear();
			final_path_from_end.clear();
			path_from_end = astar.GetPath(cur_p, tar_p, false);
			final_path_from_end = astar.ShortCutting(path_from_end);

			//����·��ѡ������·���йյ����ٵ�һ��
			final_path.clear();
			final_path_from_end.size() > final_path_from_start.size() ? final_path = final_path_from_start : final_path = final_path_from_end;
			
			//����·��
			//DrawMap(SendSocket, send_Buf, addr_debug, path_from_start);
			DrawMap(SendSocket, send_Buf, addr_debug, final_path);

		}

		final_path.pop_front();
		np = *(final_path.begin());

		

		while (!(abs(cur_p.x - np.x) < 1 && abs(cur_p.y - np.y) < 1)) {
			//��õ�ǰ��Ϣ���洢��Frame�У�ret_val��״ֵ̬���ڼ��
			GetFrame(Frame, sock_serv, msg_buf, MSG_BUF_SIZE, ip_buf, IP_BUF_SIZE, 0, addr_clt, SOCKADDR_IN_SIZE);
			astar.UpdateMap(Frame);

			real_t_vp = GetTarVel(cur_p, np, cur_ori);
			SendCommand(SendSocket, cmd_buffer, addr_recv, CONTROLLED_CAR_ID, 0, real_t_vp.vx, real_t_vp.vy);
			
			//cur_p Ϊ��ǰλ�ã�ת��Ϊ���ֵ�ͼ���ꣻ
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
