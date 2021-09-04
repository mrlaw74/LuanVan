#pragma once

#define SEND_DATA_SIZE 12
#define SEND_BUFFER_LEN (SEND_DATA_SIZE * 6)
#define REC_DATA_SIZE 12
#define REC_DATA_NUM 7
#define REC_IO_DATA_SIZE 3
#define REC_BUFFER_LEN (REC_DATA_SIZE * 6 + REC_IO_DATA_SIZE + REC_DATA_NUM)


#define ABS_LINEAR 1
#define REALATIVE_LINEAR 2
#define ABS_JOINT 3
#define REALATIVE_JOINT 4

#define ABS_LINEAR_150 7
#define ABS_LINEAR_SLOW 8
#define COMPRESSED_AIR_9 9
#define COMPRESSED_AIR_10 10

// Socket
#include <iostream>
#include<WinSock2.h>
#include <WS2tcpip.h>
#include <string>

#pragma comment (lib, "ws2_32.lib")
#include <thread>
#include <vector>z

class NachiSocket {
private:
	// Khai bao Socket
	SOCKET sock;
	SOCKET sock0;

	std::string ipAddress = "192.168.1.10";	// IP Address of the server
	int port = 48952;
	char rec_buffer[1024];

public:
	bool initializeSocket();
	bool closeSocket();
	
	bool getJointPosition(std::vector<double>& jointPosition);
	bool getJointPosition(std::vector<float>& jointPosition);
	bool getJointPosition(float jointPosition[6]);

	bool getEndEffectorPosition(std::vector<double>& endPosition);
	bool getEndEffectorPosition(std::vector<float>& endPosition);
	bool getEndEffectorPosition(float endPosition[6]);

	bool moveRobotTo(std::vector<float>& destination, int mode);
	bool moveRobotTo(std::vector<double>& destination, int mode);
	bool moveRobotTo(float destination[6], int mode);
};

/*
#pragma once

#define SEND_DATA_SIZE 12
#define SEND_BUFFER_LEN (SEND_DATA_SIZE * 6)
#define REC_DATA_SIZE 12
#define REC_DATA_NUM 7
#define REC_IO_DATA_SIZE 3
#define REC_BUFFER_LEN (REC_DATA_SIZE * 6 + REC_IO_DATA_SIZE + REC_DATA_NUM)


#define ABS_LINEAR 1
#define REALATIVE_LINEAR 2
#define ABS_JOINT 3
#define REALATIVE_JOINT 4
#define ABS_LINEAR_150 7
#define COMPRESSED_AIR_9 9
#define COMPRESSED_AIR_10 8

// Socket
#include <iostream>
#include<WinSock2.h>
#include <WS2tcpip.h>
#include <string>

#pragma comment (lib, "ws2_32.lib")
#include <thread>
#include <vector>

class NachiSocket {
private:
	// Khai bao Socket
	SOCKET sock;

	std::string ipAddress = "192.168.1.1";	// IP Address of the server
	int port = 48952;
	char rec_buffer[1024];

public:
	bool initializeSocket();
	bool closeSocket();
	
	bool getJointPosition(std::vector<double>& jointPosition);
	bool getJointPosition(std::vector<float>& jointPosition);
	bool getJointPosition(float jointPosition[6]);

	bool getEndEffectorPosition(std::vector<double>& endPosition);
	bool getEndEffectorPosition(std::vector<float>& endPosition);
	bool getEndEffectorPosition(float endPosition[6]);

	bool moveRobotTo(std::vector<float>& destination, int mode);
	bool moveRobotTo(std::vector<double>& destination, int mode);
	bool moveRobotTo(float destination[6], int mode);
};
*/