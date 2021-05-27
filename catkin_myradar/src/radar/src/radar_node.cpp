// C++
#include <stdio.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include "../include/controlcan.h" // 需要写.h文件的完整路径，..表示父目录即radar
#include <ctime>
#include <cstdlib>
#include "unistd.h"
#include <string>
// ROS
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
//custom_message
#include "radar/radar_object.h"
#include "radar/radar_objects.h"

// 大部分问题原因是权限不够，可通过进入root用户再来运行的方法解决。

VCI_BOARD_INFO pInfo; //用来获取设备信息。
int count = 0;		  //数据列表中，用来存储列表序号。
VCI_BOARD_INFO pInfo1[50];
int num = 0;
int m_run0 = 1;

ros::Publisher publisher; // ros,用来发布雷达数据
ros::Publisher v_pub;

// 通过发布Marker对象，来展示毫米波雷达数据，详情可查看visualization_mags/Marker的消息格式
visualization_msgs::Marker displayRadarObjs(radar::radar_objects objs)
{
	// 大致步骤就是先定义一个Marker对象，然后依次给Marker的各属性赋值
	visualization_msgs::Marker points;
	points.header.frame_id = "base_link";			  // 制定Marker的参考框架，即rviz中的Fixed frame
	points.header.stamp = ros::Time();				  // 时间戳
	points.ns = "radar_obj";						  // 命名空间
	points.id = 0;									  // Marker的ID号
	points.type = visualization_msgs::Marker::POINTS; // Marker的类型，有ARROW，CUBE等
	points.action = visualization_msgs::Marker::ADD;  // Marker的动作类型有ADD，DELETE等

	for (size_t i = 0; i < objs.num; i++)
	{
		// 将毫米波雷达的点xy值赋给geometry_msgs/Point对象
		geometry_msgs::Point p;
		p.x = objs.radar_obj.at(i).x;
		p.y = objs.radar_obj.at(i).y;
		p.z = 0;
		// 定义点的颜色，为绿色，a为不透明度，0为透明，1为不透明
		std_msgs::ColorRGBA green;
		green.a = 1.0;
		green.b = 0.0;
		green.r = 0.0;
		green.g = 1.0;
		points.color = green;
		// 点的尺寸
		points.scale.x = 0.2;
		points.scale.y = 0.2;
		points.scale.z = 0.2;
		points.frame_locked = true;
		points.points.push_back(p); // 将p的xy坐标添加到Marker对象points里
	}
	return points;
}

// 过滤点，只留前方一个矩形范围内的点
void filter_obj(radar::radar_objects &objs)
{
	for (auto ite = objs.radar_obj.begin(); ite != objs.radar_obj.end();)
	{
		// if(ite->x > 14.0 || (ite->x < 0.01 && ite->y < 0.01)) // 阈值设置为14.0，根据实际情况修改
		// 前方10米以外或者1厘米以内或左侧3米以外或右侧3米以外的点剔除掉
		if (ite->x > 10.0 || ite->x < 0.01 || ite->y > 3 || ite->y < -3)
		{
			ite = objs.radar_obj.erase(ite);
			objs.num--;
		}
		else
			ite++;
	}
}

// 障碍物检测
void objects_detection(float x, float y)
{
	if (x > 0.01 && x < 2)
	{
		printf("front!!-----,x = %.2f,y = %.2f-----\n", x, y);
	}
}

void *receive_func(void *param) //接收线程。
{
	int reclen = 0;
	VCI_CAN_OBJ rec[3000]; //接收缓存，设为3000为佳。
	int i, j;

	int *run = (int *)param; //线程启动，退出控制。
	int ind = 0;
	radar::radar_objects objs; // 雷达ros消息
	int index1 = 0;			   // for 0x60B
	int index2 = 0;			   // for 0x60C

	printf("--------Start--------\n");

	while ((*run) & 0x0f)
	{
		if ((reclen = VCI_Receive(VCI_USBCAN2, 0, ind, rec, 3000, 100)) > 0) //调用接收函数，如果有数据，进行数据处理显示。
		{
			objs.radar_obj.clear();
			index1 = 0;
			index2 = 0;
			for (j = 0; j < reclen; j++)
			{
				// printf("Index:%04d  ", count++);	// 打印序号，序号递增
				// printf("CAN%d RX ID:0x%08X", ind + 1, rec[j].ID); //帧ID

				// 根据CAN信息帧里的两个标志位来打印标准帧or扩展帧，数据帧or远程帧
				// rec[j].ExternFlag == 0 ? printf(" Standard ") : printf(" Extend ");
				// rec[j].RemoteFlag == 0 ? printf(" Data ") : printf(" Remote ");

				// if (rec[j].ExternFlag == 0)
				// 	printf(" Standard "); //帧格式：标准帧
				// if (rec[j].ExternFlag == 1)
				// 	printf(" Extend   "); //帧格式：扩展帧
				// if (rec[j].RemoteFlag == 0)
				// 	printf(" Data   "); //帧类型：数据帧
				// if (rec[j].RemoteFlag == 1)
				// 	printf(" Remote ");	//帧类型：远程帧

				// printf("DLC:0x%02X", rec[j].DataLen); //帧长度
				// printf(" data:0x");		//数据，原始16进制的数据
				// for (i = 0; i < rec[j].DataLen; i++)
				// {
				// 	printf(" %02X", rec[j].Data[i]);
				// }
				// printf(" TimeStamp:0x%08X", rec[j].TimeStamp); //时间标识。

				// rec在上面的定义是一个VCI_CAN_OBJ数组，其中VCI_CAN_OBJ结构体的ID这里做了自定义
				// ID=0x60A代表头部，ID=0x60B代表数据，ID=0x60C代表置信度
				if (rec[j].ID == 0x60A) // header
				{
					BYTE obj_num_t = rec[j].Data[0]; // 目标数量
					objs.num = obj_num_t;
					// printf(" nums:%d ", obj_num_t);
					for (unsigned int ii = 0; ii < obj_num_t; ii++)
					{
						radar::radar_object obj;
						objs.radar_obj.push_back(obj);
					}
				}
				else if (rec[j].ID == 0x60B)
				{
					BYTE obj_id = rec[j].Data[0]; // 目标ID

					int a = ((int)rec[j].Data[1]) << 5;
					int b = (rec[j].Data[2] & 0xf8) >> 3;
					float x = (a + b) * 0.2 - 500;
					// printf(" x:%.2f \n", x); // 打印x坐标
					a = ((int)(rec[j].Data[2] & 0x07)) << 8;
					b = rec[j].Data[3];
					float y = (a + b) * 0.2 - 204.6; // 单位m, 0.2 为res,-204.6 为offset,计算公式：十进制值 × res + offset.
					// printf("y:%.2f \n", y);			 // 打印y坐标
					
					objects_detection(x,y);	// 障碍物检测

					if (objs.radar_obj.size() > index1)
					{
						objs.radar_obj[index1].id = (unsigned int)obj_id;
						objs.radar_obj[index1].x = x;
						objs.radar_obj[index1].y = y;
						index1++;
					}
				}
				else if (rec[j].ID == 0x60C)
				{
					BYTE data = (rec[j].Data[6] & 0xe0) >> 5;
					float prob = 0.0;
					switch (data)
					{
					case 0x00:
						prob = 0.0;
						break;
					case 0x01:
						prob = 0.25;
						break;
					case 0x02:
						prob = 0.50;
						break;
					case 0x03:
						prob = 0.75;
						break;
					case 0x04:
						prob = 0.90;
						break;
					case 0x05:
						prob = 0.99;
						break;
					case 0x06:
						prob = 0.999;
						break;
					case 0x07:
						prob = 1.0;
						break;
					default:
						break;
					}
					// printf("data:%.2f", data);
					// printf(" prob:%.2f ", prob); // 打印置信度，即该点多大概率为障碍物
					if (objs.radar_obj.size() > index2)
					{
						objs.radar_obj[index2].prob = prob;
						objs.radar_obj[index2].velocity_x = 0;
						objs.radar_obj[index2].velocity_y = 0;
						index2++;
					}
				}
				// printf("\n");
			}
			objs.header.frame_id = std::string("radar");
			ros::Time time_ = ros::Time::now();
			objs.header.stamp = time_;
			if (objs.radar_obj.size() == objs.num) // 不知道为什么会有一些空的消息，所以这里做个筛选
			{
				filter_obj(objs);
				if (objs.radar_obj.size() > 0)
				{
					publisher.publish(objs);

					v_pub.publish(displayRadarObjs(objs));
				}
			}
		}
		ind = !ind; //变换通道号，以便下次读取另一通道，交替读取。
		usleep(10000);
	}
	printf("run thread exit\n"); //退出接收线程
	pthread_exit(0);
}

void handler(int sig)
{
	m_run0 = 0;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "radar");
	ros::NodeHandle nh("~");
	std::string output_topic = "radar_objects_list";
	// 定义发布者
	publisher = nh.advertise<radar::radar_objects>(output_topic, 100, true);
	v_pub = nh.advertise<visualization_msgs::Marker>("radar_objs", 0);

	printf(">>this is hello !\r\n"); //指示程序已运行

	num = VCI_FindUsbDevice2(pInfo1); // 需启动底层USB驱动

	printf(">>USBCAN DEVICE NUM:");
	printf("%d", num);
	printf(" PCS");
	printf("\n");

	for (int i = 0; i < num; i++)
	{
		printf("Device:");
		printf("%d", i);
		printf("\n");
		printf(">>Get VCI_ReadBoardInfo success!\n");

		printf(">>Serial_Num:%c", pInfo1[i].str_Serial_Num[0]);
		printf("%c", pInfo1[i].str_Serial_Num[1]);
		printf("%c", pInfo1[i].str_Serial_Num[2]);
		printf("%c", pInfo1[i].str_Serial_Num[3]);
		printf("%c", pInfo1[i].str_Serial_Num[4]);
		printf("%c", pInfo1[i].str_Serial_Num[5]);
		printf("%c", pInfo1[i].str_Serial_Num[6]);
		printf("%c", pInfo1[i].str_Serial_Num[7]);
		printf("%c", pInfo1[i].str_Serial_Num[8]);
		printf("%c", pInfo1[i].str_Serial_Num[9]);
		printf("%c", pInfo1[i].str_Serial_Num[10]);
		printf("%c", pInfo1[i].str_Serial_Num[11]);
		printf("%c", pInfo1[i].str_Serial_Num[12]);
		printf("%c", pInfo1[i].str_Serial_Num[13]);
		printf("%c", pInfo1[i].str_Serial_Num[14]);
		printf("%c", pInfo1[i].str_Serial_Num[15]);
		printf("%c", pInfo1[i].str_Serial_Num[16]);
		printf("%c", pInfo1[i].str_Serial_Num[17]);
		printf("%c", pInfo1[i].str_Serial_Num[18]);
		printf("%c", pInfo1[i].str_Serial_Num[19]);
		printf("\n");

		printf(">>hw_Type:%c", pInfo1[i].str_hw_Type[0]);
		printf("%c", pInfo1[i].str_hw_Type[1]);
		printf("%c", pInfo1[i].str_hw_Type[2]);
		printf("%c", pInfo1[i].str_hw_Type[3]);
		printf("%c", pInfo1[i].str_hw_Type[4]);
		printf("%c", pInfo1[i].str_hw_Type[5]);
		printf("%c", pInfo1[i].str_hw_Type[6]);
		printf("%c", pInfo1[i].str_hw_Type[7]);
		printf("%c", pInfo1[i].str_hw_Type[8]);
		printf("%c", pInfo1[i].str_hw_Type[9]);
		printf("\n");

		printf(">>Firmware Version:V");
		printf("%x", (pInfo1[i].fw_Version & 0xF00) >> 8);
		printf(".");
		printf("%x", (pInfo1[i].fw_Version & 0xF0) >> 4);
		printf("%x", pInfo1[i].fw_Version & 0xF);
		printf("\n");
	}
	printf(">>\n");
	printf(">>\n");
	printf(">>\n");

	// 设备第一次Open的时候返回1，第二次Open的时候返回0，因为不能Open一个已经开启的设备
	// printf("OpenDevice return:%d\n", VCI_OpenDevice(VCI_USBCAN2, 0, 0));
	// printf("VCI_USBCAN2:%d\n",VCI_USBCAN2); // 打印设备型号，购买的型号为4

	// 打开设备，参数一是设备型号，参数二是设备索引，第一台设备是0，第二台是1，以此类推，参数三为保留参数，默认0
	if (VCI_OpenDevice(VCI_USBCAN2, 0, 0) == 1)
	{
		printf(">>open deivce success!\n"); //打开设备成功
	}
	else
	{
		printf(">>open deivce error!\n");
		exit(1);
	}

	if (VCI_ReadBoardInfo(VCI_USBCAN2, 0, &pInfo) == 1) //读取设备序列号、版本等信息。
	{
		printf(">>Get VCI_ReadBoardInfo success!\n");

		//printf(" %08X", pInfo.hw_Version);printf("\n");
		//printf(" %08X", pInfo.fw_Version);printf("\n");
		//printf(" %08X", pInfo.dr_Version);printf("\n");
		//printf(" %08X", pInfo.in_Version);printf("\n");
		//printf(" %08X", pInfo.irq_Num);printf("\n");
		//printf(" %08X", pInfo.can_Num);printf("\n");
		printf(">>Serial_Num:%c", pInfo.str_Serial_Num[0]);
		printf("%c", pInfo.str_Serial_Num[1]);
		printf("%c", pInfo.str_Serial_Num[2]);
		printf("%c", pInfo.str_Serial_Num[3]);
		printf("%c", pInfo.str_Serial_Num[4]);
		printf("%c", pInfo.str_Serial_Num[5]);
		printf("%c", pInfo.str_Serial_Num[6]);
		printf("%c", pInfo.str_Serial_Num[7]);
		printf("%c", pInfo.str_Serial_Num[8]);
		printf("%c", pInfo.str_Serial_Num[9]);
		printf("%c", pInfo.str_Serial_Num[10]);
		printf("%c", pInfo.str_Serial_Num[11]);
		printf("%c", pInfo.str_Serial_Num[12]);
		printf("%c", pInfo.str_Serial_Num[13]);
		printf("%c", pInfo.str_Serial_Num[14]);
		printf("%c", pInfo.str_Serial_Num[15]);
		printf("%c", pInfo.str_Serial_Num[16]);
		printf("%c", pInfo.str_Serial_Num[17]);
		printf("%c", pInfo.str_Serial_Num[18]);
		printf("%c", pInfo.str_Serial_Num[19]);
		printf("\n");

		printf(">>hw_Type:%c", pInfo.str_hw_Type[0]);
		printf("%c", pInfo.str_hw_Type[1]);
		printf("%c", pInfo.str_hw_Type[2]);
		printf("%c", pInfo.str_hw_Type[3]);
		printf("%c", pInfo.str_hw_Type[4]);
		printf("%c", pInfo.str_hw_Type[5]);
		printf("%c", pInfo.str_hw_Type[6]);
		printf("%c", pInfo.str_hw_Type[7]);
		printf("%c", pInfo.str_hw_Type[8]);
		printf("%c", pInfo.str_hw_Type[9]);
		printf("\n");

		printf(">>Firmware Version:V");
		printf("%x", (pInfo.fw_Version & 0xF00) >> 8);
		printf(".");
		printf("%x", (pInfo.fw_Version & 0xF0) >> 4);
		printf("%x", pInfo.fw_Version & 0xF);
		printf("\n");
	}
	else
	{
		printf(">>Get VCI_ReadBoardInfo error!\n");
		exit(1);
	}

	//初始化参数，严格参照二次开发函数库说明书。
	VCI_INIT_CONFIG config;
	config.AccCode = 0;
	config.AccMask = 0xFFFFFFFF;
	config.Filter = 1;	   //接收所有帧
	config.Timing0 = 0x00; /*波特率125 Kbps  0x03  0x1C,500 Kbps 0x00 0x1C*/
	config.Timing1 = 0x1C;
	config.Mode = 0; //正常模式

	if (VCI_InitCAN(VCI_USBCAN2, 0, 0, &config) != 1)
	{
		printf(">>Init CAN1 error\n");
		VCI_CloseDevice(VCI_USBCAN2, 0);
	}

	if (VCI_StartCAN(VCI_USBCAN2, 0, 0) != 1)
	{
		printf(">>Start CAN1 error\n");
		VCI_CloseDevice(VCI_USBCAN2, 0);
	}

	if (VCI_InitCAN(VCI_USBCAN2, 0, 1, &config) != 1)
	{
		printf(">>Init can2 error\n");
		VCI_CloseDevice(VCI_USBCAN2, 0);
	}
	if (VCI_StartCAN(VCI_USBCAN2, 0, 1) != 1)
	{
		printf(">>Start can2 error\n");
		VCI_CloseDevice(VCI_USBCAN2, 0);
	}
	//需要发送的帧，结构体设置
	VCI_CAN_OBJ send[1];
	send[0].ID = 0;
	send[0].SendType = 0;
	send[0].RemoteFlag = 0;
	send[0].ExternFlag = 1;
	send[0].DataLen = 8;

	int i = 0;
	for (i = 0; i < send[0].DataLen; i++)
	{
		send[0].Data[i] = i;
	}

	// 创建一个线程对象
	pthread_t threadid;
	int ret;

	// 创建新线程，进入receive_func接收函数
	ret = pthread_create(&threadid, NULL, receive_func, &m_run0);

	int times = 5;
	while (times--)
	{
		if (VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1)
		{
			printf("Index:%04d  ", count);
			count++;
			printf("CAN1 TX ID:0x%08X", send[0].ID);
			if (send[0].ExternFlag == 0)
				printf(" Standard ");
			if (send[0].ExternFlag == 1)
				printf(" Extend   ");
			if (send[0].RemoteFlag == 0)
				printf(" Data   ");
			if (send[0].RemoteFlag == 1)
				printf(" Remote ");
			printf("DLC:0x%02X", send[0].DataLen);
			printf(" data:0x");

			for (i = 0; i < send[0].DataLen; i++)
			{
				printf(" %02X", send[0].Data[i]);
			}

			printf("\n");
			send[0].ID += 1;
		}
		else
		{
			break;
		}

		if (VCI_Transmit(VCI_USBCAN2, 0, 1, send, 1) == 1)
		{
			printf("Index:%04d  ", count);
			count++;
			printf("CAN2 TX ID:0x%08X", send[0].ID);
			if (send[0].ExternFlag == 0)
				printf(" Standard ");
			if (send[0].ExternFlag == 1)
				printf(" Extend   ");
			if (send[0].RemoteFlag == 0)
				printf(" Data   ");
			if (send[0].RemoteFlag == 1)
				printf(" Remote ");
			printf("DLC:0x%02X", send[0].DataLen);
			printf(" data:0x");
			for (i = 0; i < send[0].DataLen; i++)
			{
				printf(" %02X", send[0].Data[i]);
			}
			printf("\n");
			send[0].ID += 1;
		}
		else
			break;
	}
	// usleep(10000000);//延时单位us，这里设置 10 000 000=10s    10s后关闭接收线程，并退出主程序。
	// m_run0=0;//线程关闭指令。
	signal(SIGINT, handler);
	pthread_join(threadid, NULL);	 //等待线程关闭。
	usleep(100000);					 //延时100ms。
	VCI_ResetCAN(VCI_USBCAN2, 0, 0); //复位CAN1通道。
	usleep(100000);					 //延时100ms。
	VCI_ResetCAN(VCI_USBCAN2, 0, 1); //复位CAN2通道。
	usleep(100000);					 //延时100ms。
	VCI_CloseDevice(VCI_USBCAN2, 0); //关闭设备。
	//除收发函数外，其它的函数调用前后，最好加个毫秒级的延时，即不影响程序的运行，又可以让USBCAN设备有充分的时间处理指令。
	//goto ext;
	if (m_run0 != 0)
		ros::spin();
}
