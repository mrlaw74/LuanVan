#include "NachiSocket.h"

bool NachiSocket::initializeSocket() {
	// Khoi tao socket client

	WSAData data;
	WORD ver = MAKEWORD(2, 0);	//version 2.0
	int wsResult = WSAStartup(ver, &data);
	if (wsResult != 0)
	{
		std::cerr << "Can't start Winsock, Err " << wsResult << std::endl;
		system("pause");
		return false;
	}

	//Make server socket
	sock0 = socket(AF_INET, SOCK_STREAM, 0); 
	if (sock0 == INVALID_SOCKET)  
	{  
		std::cout << "Can't create socket, Err # : " << WSAGetLastError() << std::endl;  
		return false; 
	} 
 
	// Fill in a hint structure
	sockaddr_in address;
	address.sin_family = AF_INET;
	address.sin_port = htons(port);
	address.sin_addr.S_un.S_addr = INADDR_ANY; 

	if (bind(sock0, (struct sockaddr *)&address, sizeof(address)) != 0)  
	{  
		std::cout << "bind : " << WSAGetLastError() << std::endl;  
		return false; 
	} 

	//Communicate to the Client 
	if (listen(sock0, 5) != 0)  
	{  
		std::cout << "listen : " << WSAGetLastError() << std::endl;  
		return false; 
	} 
 
	int len; 
	sockaddr_in client;
	len = sizeof(client); 
	std::cout << "Server (PC) is waitting connection with Client (CFD Controller)..." << std::endl; 
	sock = accept(sock0, (struct sockaddr *)&client, &len); 
	if(sock == INVALID_SOCKET)
	{  
		std::cout << "accept : " << WSAGetLastError() << std::endl;  
		return false; 
	} 

	return true;
};

bool NachiSocket::closeSocket() {
	send(sock, "C", 1, 0);

	// shutdown the send half of the connection since no more data will be sent
	shutdown(sock, SD_SEND);
	//Cleanup
	closesocket(sock);
	WSACleanup();// Cleanup winsoc
	return true;
};

bool NachiSocket::getJointPosition(std::vector<double>& jointPosition)
{
	float jointPositionArray[6];
	bool check = getJointPosition(jointPositionArray);
	jointPosition.clear();
	for (int i = 0; i < 6; ++i)
		jointPosition.push_back(static_cast<double>(jointPositionArray[i]));
	return check;
}

bool NachiSocket::getJointPosition(std::vector<float>& jointPosition)
{
	float jointPositionArray[6];
	bool check = getJointPosition(jointPositionArray);
	jointPosition.clear();
	for (int i = 0; i < 6; ++i)
		jointPosition.push_back(jointPositionArray[i]);
	return check;
}

bool NachiSocket::getJointPosition(float jointPosition[6])
{
	int check = 0;
	char* temp = NULL;
	char* ctx = NULL;
	char rec_buffer[REC_BUFFER_LEN + 1] = { 0 };
	send(sock, "J", 1, 0);
	//Recieve
	check = recv(sock, rec_buffer, sizeof(rec_buffer), 0);
	if ((rec_buffer[0] != 'R') || (rec_buffer[1] != 'E') || (rec_buffer[2] != 'A') || (check < 1)) {
		std::cout << "recv : " << WSAGetLastError() << std::endl;
		return false;
	}

	check = recv(sock, rec_buffer, sizeof(rec_buffer), 0);

	jointPosition[0] = atof(strtok_s(rec_buffer, ",", &ctx));
	jointPosition[1] = atof(strtok_s(NULL, ",", &ctx));
	jointPosition[2] = atof(strtok_s(NULL, ",", &ctx));
	jointPosition[3] = atof(strtok_s(NULL, ",", &ctx));
	jointPosition[4] = atof(strtok_s(NULL, ",", &ctx));
	jointPosition[5] = atof(strtok_s(NULL, ",", &ctx));
	/*std::cout << "Gia tri cac goc hien tai la : " << std::endl;*/
	std::cout << "Joint 1 : " << jointPosition[0] << std::endl;
	std::cout << "Joint 2 : " << jointPosition[1] << std::endl;
	std::cout << "Joint 3 : " << jointPosition[2] << std::endl;
	std::cout << "Joint 4 : " << jointPosition[3] << std::endl;
	std::cout << "Joint 5 : " << jointPosition[4] << std::endl;
	std::cout << "Joint 6 : " << jointPosition[5] << std::endl;

	return true;
}

bool NachiSocket::getEndEffectorPosition(std::vector<double>& endPosition)
{
	float endPositionArray[6];
	bool check = getEndEffectorPosition(endPositionArray);
	endPosition.clear();
	for (int i = 0; i < 6; ++i)
		endPosition.push_back(static_cast<double>(endPositionArray[i]));
	return check;
}

bool NachiSocket::getEndEffectorPosition(std::vector<float>& endPosition)
{
	float endPositionArray[6];
	bool check = getEndEffectorPosition(endPositionArray);
	endPosition.clear();
	for (int i = 0; i < 6; ++i)
		endPosition.push_back(endPositionArray[i]);
	return check;
}

bool NachiSocket::getEndEffectorPosition(float endPosition[6])
{
	int check = 0;
	char* temp = NULL;
	char* ctx = NULL;
	char rec_buffer[REC_BUFFER_LEN + 1] = { 0 };
	send(sock, "P", 1, 0);

	//Recieve
	check = recv(sock, rec_buffer, sizeof(rec_buffer), 0);
	//std::cout << rec_buffer << std::endl; // ready signal
	if ((rec_buffer[0] != 'R') || (rec_buffer[1] != 'E') || (rec_buffer[2] != 'A') || (check < 1)) 
	{
		std::cout << "recv : " << WSAGetLastError() << std::endl;
		return false;
	}

	check = recv(sock, rec_buffer, sizeof(rec_buffer), 0);

	endPosition[0] = atof(strtok_s(rec_buffer, ",", &ctx));
	endPosition[1] = atof(strtok_s(NULL, ",", &ctx));
	endPosition[2] = atof(strtok_s(NULL, ",", &ctx));
	endPosition[3] = atof(strtok_s(NULL, ",", &ctx));
	endPosition[4] = atof(strtok_s(NULL, ",", &ctx));
	endPosition[5] = atof(strtok_s(NULL, ",", &ctx));

	std::cout << "X     : " << endPosition[0] << std::endl;
	std::cout << "Y     : " << endPosition[1] << std::endl;
	std::cout << "Z     : " << endPosition[2] << std::endl;
	std::cout << "Roll  : " << endPosition[3] << std::endl;
	std::cout << "Pitch : " << endPosition[4] << std::endl;
	std::cout << "Yaw   : " << endPosition[5] << std::endl;

	return true;
}

bool NachiSocket::moveRobotTo(std::vector<float>& destination, int mode)
{
	float destinationArray[6];
	for (int i = 0; i < 6; ++i)
		destinationArray[i] = destination[i];
	bool check = moveRobotTo(destinationArray, mode);
	return check;
}

bool NachiSocket::moveRobotTo(std::vector<double>& destination, int mode)
{
	float destinationArray[6];
	for (int i = 0; i < 6; ++i)
		destinationArray[i] = static_cast<float>(destination[i]);
	bool check = moveRobotTo(destinationArray, mode);
	return check;
}

bool NachiSocket::moveRobotTo(float destination[6], int mode)
{
	char send_buffer[SEND_BUFFER_LEN + 3];

	char shift_x[] = "00000.00";
	char shift_y[] = "00000.00";
	char shift_z[] = "00000.00";
	char shift_r[] = "00000.00";
	char shift_p[] = "00000.00";
	char shift_ya[] = "00000.00";
	char step[] = "  0";
	char rec_buffer[REC_BUFFER_LEN + 1] = { 0 };
	int check = 0;

	do
	{
		send(sock, "A", 1, 0);
		memset(rec_buffer, 0, sizeof rec_buffer);
		check = recv(sock, rec_buffer, sizeof(rec_buffer), 0);
		//std::cout << rec_buffer << std::endl; // ready signal
		
	} while ((rec_buffer[0] != 'R') || (rec_buffer[1] != 'E') || (rec_buffer[2] != 'A') || (check < 1));
	sprintf_s(shift_x, "%08.2f", destination[0]);
	sprintf_s(shift_y, "%08.2f", destination[1]);
	sprintf_s(shift_z, "%08.2f", destination[2]);
	sprintf_s(shift_r, "%08.2f", destination[3]);
	sprintf_s(shift_p, "%08.2f", destination[4]);
	sprintf_s(shift_ya, "%08.2f", destination[5]);
	sprintf_s(step, "%3d", mode);
	sprintf_s(send_buffer, "%s%s%s%s%s%s%s", shift_x, shift_y, shift_z, shift_r, shift_p, shift_ya, step);

	check = 0;
	do
	{
		check = send(sock, send_buffer, sizeof(send_buffer), 0);
		memset(rec_buffer, 0, sizeof rec_buffer);
		check = recv(sock, rec_buffer, sizeof(rec_buffer), 0);

	} while (rec_buffer[0] != 'F' || rec_buffer[1] != 'I' || rec_buffer[2] != 'N' || check < 1);
	//std::cout << rec_buffer << std::endl;
	return check;
}

/*
#include "NachiSocket.h"

bool NachiSocket::initializeSocket() {
	// Khoi tao socket client

	WSAData data;
	WORD ver = MAKEWORD(2, 0);	//version 2.0
	int wsResult = WSAStartup(ver, &data);
	if (wsResult != 0)
	{
		std::cerr << "Can't start Winsock, Err " << wsResult << std::endl;
		system("pause");
		return false;
	}
	// Create client socket
	sock = socket(AF_INET, SOCK_STREAM, 0);
	if (sock == INVALID_SOCKET)
	{
		std::cerr << "Can't create socket, Err #" << WSAGetLastError() << std::endl;
		WSACleanup();
		system("pause");
		return false;
	}
	// Fill in a hint structure
	sockaddr_in address;
	address.sin_family = AF_INET;
	address.sin_port = htons(port);
	inet_pton(AF_INET, ipAddress.c_str(), &address.sin_addr);


	// Connect to server
	int connResult = connect(sock, (sockaddr*)&address, sizeof(address));
	if (connResult == SOCKET_ERROR)
	{
		std::cerr << "Can't connect to server, Err " << WSAGetLastError() << std::endl;
		closesocket(sock);
		WSACleanup();
		return false;
	}
	else
		//std::cout << "Connect successfully to " << ipAddress << std::endl;

	return true;
};

bool NachiSocket::closeSocket() {
	send(sock, "C", 1, 0);

	// shutdown the send half of the connection since no more data will be sent
	shutdown(sock, SD_SEND);
	//Cleanup
	closesocket(sock);
	WSACleanup();// Cleanup winsoc
	return true;
};

bool NachiSocket::getJointPosition(std::vector<double>& jointPosition)
{
	float jointPositionArray[6];
	bool check = getJointPosition(jointPositionArray);
	jointPosition.clear();
	for (int i = 0; i < 6; ++i)
		jointPosition.push_back(static_cast<double>(jointPositionArray[i]));
	return check;
}

bool NachiSocket::getJointPosition(std::vector<float>& jointPosition)
{
	float jointPositionArray[6];
	bool check = getJointPosition(jointPositionArray);
	jointPosition.clear();
	for (int i = 0; i < 6; ++i)
		jointPosition.push_back(jointPositionArray[i]);
	return check;
}

bool NachiSocket::getJointPosition(float jointPosition[6])
{
	int check = 0;
	char* temp = NULL;
	char* ctx = NULL;
	char rec_buffer[REC_BUFFER_LEN + 1] = { 0 };
	send(sock, "J", 1, 0);
	//Recieve
	check = recv(sock, rec_buffer, sizeof(rec_buffer), 0);
	if ((rec_buffer[0] != 'R') || (rec_buffer[1] != 'E') || (rec_buffer[2] != 'A') || (check < 1)) {
		std::cout << "recv : " << WSAGetLastError() << std::endl;
		return false;
	}

	check = recv(sock, rec_buffer, sizeof(rec_buffer), 0);

	jointPosition[0] = atof(strtok_s(rec_buffer, ",", &ctx));
	jointPosition[1] = atof(strtok_s(NULL, ",", &ctx));
	jointPosition[2] = atof(strtok_s(NULL, ",", &ctx));
	jointPosition[3] = atof(strtok_s(NULL, ",", &ctx));
	jointPosition[4] = atof(strtok_s(NULL, ",", &ctx));
	jointPosition[5] = atof(strtok_s(NULL, ",", &ctx));

	//std::cout << "Joint 1 : " << jointPosition[0] << std::endl;
	//std::cout << "Joint 2 : " << jointPosition[1] << std::endl;
	//std::cout << "Joint 3 : " << jointPosition[2] << std::endl;
	//std::cout << "Joint 4 : " << jointPosition[3] << std::endl;
	//std::cout << "Joint 5 : " << jointPosition[4] << std::endl;
	//std::cout << "Joint 6 : " << jointPosition[5] << std::endl;

	return true;
}

bool NachiSocket::getEndEffectorPosition(std::vector<double>& endPosition)
{
	float endPositionArray[6];
	bool check = getEndEffectorPosition(endPositionArray);
	endPosition.clear();
	for (int i = 0; i < 6; ++i)
		endPosition.push_back(static_cast<double>(endPositionArray[i]));
	return check;
}

bool NachiSocket::getEndEffectorPosition(std::vector<float>& endPosition)
{
	float endPositionArray[6];
	bool check = getEndEffectorPosition(endPositionArray);
	endPosition.clear();
	for (int i = 0; i < 6; ++i)
		endPosition.push_back(endPositionArray[i]);
	return check;
}

bool NachiSocket::getEndEffectorPosition(float endPosition[6])
{
	int check = 0;
	char* temp = NULL;
	char* ctx = NULL;
	char rec_buffer[REC_BUFFER_LEN + 1] = { 0 };
	send(sock, "P", 1, 0);

	//Recieve
	check = recv(sock, rec_buffer, sizeof(rec_buffer), 0);
	//std::cout << rec_buffer << std::endl; // ready signal
	if ((rec_buffer[0] != 'R') || (rec_buffer[1] != 'E') || (rec_buffer[2] != 'A') || (check < 1)) {
		std::cout << "recv : " << WSAGetLastError() << std::endl;
		return false;
	}

	check = recv(sock, rec_buffer, sizeof(rec_buffer), 0);

	endPosition[0] = atof(strtok_s(rec_buffer, ",", &ctx));
	endPosition[1] = atof(strtok_s(NULL, ",", &ctx));
	endPosition[2] = atof(strtok_s(NULL, ",", &ctx));
	endPosition[3] = atof(strtok_s(NULL, ",", &ctx));
	endPosition[4] = atof(strtok_s(NULL, ",", &ctx));
	endPosition[5] = atof(strtok_s(NULL, ",", &ctx));

	//std::cout << "X     : " << endPosition[0] << std::endl;
	//std::cout << "Y     : " << endPosition[1] << std::endl;
	//std::cout << "Z     : " << endPosition[2] << std::endl;
	//std::cout << "Roll  : " << endPosition[3] << std::endl;
	//std::cout << "Pitch : " << endPosition[4] << std::endl;
	//std::cout << "Yaw   : " << endPosition[5] << std::endl;

	return true;
}

bool NachiSocket::moveRobotTo(std::vector<float>& destination, int mode)
{
	float destinationArray[6];
	for (int i = 0; i < 6; ++i)
		destinationArray[i] = destination[i];
	bool check = moveRobotTo(destinationArray, mode);
	return check;
}

bool NachiSocket::moveRobotTo(std::vector<double>& destination, int mode)
{
	float destinationArray[6];
	for (int i = 0; i < 6; ++i)
		destinationArray[i] = static_cast<float>(destination[i]);
	bool check = moveRobotTo(destinationArray, mode);
	return check;
}

bool NachiSocket::moveRobotTo(float destination[6], int mode)
{
	char send_buffer[SEND_BUFFER_LEN + 3];

	char shift_x[] = "00000.00";
	char shift_y[] = "00000.00";
	char shift_z[] = "00000.00";
	char shift_r[] = "00000.00";
	char shift_p[] = "00000.00";
	char shift_ya[] = "00000.00";
	char step[] = "  0";
	char rec_buffer[REC_BUFFER_LEN + 1] = { 0 };
	int check = 0;

	send(sock, "A", 1, 0);
	do
	{
		
		memset(rec_buffer, 0, sizeof rec_buffer);
		check = recv(sock, rec_buffer, sizeof(rec_buffer), 0);
		//std::cout << rec_buffer << std::endl; // ready signal
		
	} while ((rec_buffer[0] != 'R') || (rec_buffer[1] != 'E') || (rec_buffer[2] != 'A') || (check < 1));
	sprintf_s(shift_x, "%08.2f", destination[0]);
	sprintf_s(shift_y, "%08.2f", destination[1]);
	sprintf_s(shift_z, "%08.2f", destination[2]);
	sprintf_s(shift_r, "%08.2f", destination[3]);
	sprintf_s(shift_p, "%08.2f", destination[4]);
	sprintf_s(shift_ya, "%08.2f", destination[5]);
	sprintf_s(step, "%3d", mode);
	sprintf_s(send_buffer, "%s%s%s%s%s%s%s", shift_x, shift_y, shift_z, shift_r, shift_p, shift_ya, step);

	check = send(sock, send_buffer, sizeof(send_buffer), 0);
	//std::cout << send_buffer << std::endl;

	check = 0;
	do
	{
		memset(rec_buffer, 0, sizeof rec_buffer);
		check = recv(sock, rec_buffer, sizeof(rec_buffer), 0);

	} while (rec_buffer[0] != 'F' || rec_buffer[1] != 'I' || rec_buffer[2] != 'N' || check < 1);
	//std::cout << rec_buffer << std::endl;
	return check;
}

*/