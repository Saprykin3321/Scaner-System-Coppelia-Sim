#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include "Network.h"
#include <iostream>

#pragma comment(lib, "ws2_32.lib")

//// U D P ////

Server_UDP::Server_UDP()
{
	// Must be initialized as follows, otherwise socket() will return 10093 error
	// Initialize WSA
	WORD sockVersion = MAKEWORD(2, 2);
	if (WSAStartup(sockVersion, &wsaData) != 0) //Initialize ws2_32.dll through a process
	{
		std::cout << "SERVER_UDP: Initialize WSA failed" << std::endl;
		return;
	}
	// Initialize the UDDP socket
	m_sServer = socket(AF_INET, SOCK_DGRAM, 0);

	m_SocAddrserver.sin_addr.S_un.S_addr = inet_addr(address);
	m_SocAddrserver.sin_family = AF_INET;
	m_SocAddrserver.sin_port = htons(port_udp);

	int ret = bind(m_sServer, (sockaddr*)&m_SocAddrserver, sizeof(m_SocAddrserver));
	if (ret == -1)
	{
		std::cout << "SERVER_UDP: bind failed" << std::endl;
		WSACleanup();
	}
	else
	{
		std::cout << "SERVER_UDP: Succes bind. Waiting hello message" << std::endl;
		int len_Client = sizeof(sockaddr);
		char recBuf[1025];
		int len = recvfrom(m_sServer, recBuf, 1024, 0, (sockaddr*)&m_SocAddrClient, &len_Client);
		if (len > 0)
		{
			recBuf[len] = '\0';
			std::cout << "SERVER_UDP: Hello message from client: " << recBuf << std::endl;
		}
	}
	std::cout << "SERVER_UDP: Succes connected" << std::endl;
}

void Server_UDP::SendMsg(const char* sendBuf, int size)
{
	int ret = sendto(m_sServer, sendBuf, size, 0, (sockaddr*)&m_SocAddrClient, sizeof(m_SocAddrClient));
	if (ret == -1)
	{
		std::cout << "SERVER_UDP: Message is not sending" << WSAGetLastError() << std::endl;
	}
}

void Server_UDP::RecMsg(char *recBuf)
{
	//std::cout << "Begin rec...(server)" << std::endl;
	int len = recvfrom(m_sServer, recBuf, 1024, 0, 0, 0);
	if (len > 0)
	{
		recBuf[len] = '\0';
	}
}

void Server_UDP::close()
{
	closesocket(m_sServer);
	WSACleanup();
}

//Server_UDP::~Server_UDP()
//{
//	closesocket(m_sServer);
//	WSACleanup();
//}

//// T C P ////
Server_TCP::Server_TCP()
{
	WORD DLLVersion = MAKEWORD(2, 2);
	if (WSAStartup(DLLVersion, &wsaData) != 0)
	{
		std::cout << "SERVER_TCP: Initialize WSA failed" << std::endl;
		exit(1);
	}

	int sizeofaddr = sizeof(addr);

	server = socket(AF_INET, SOCK_STREAM, 0);
	if (server < 0)
	{
		std::cout << "SERVER_TCP: Server socket error" << std::endl;
		exit(0);
	}
	std::cout << "SERVER_TCP: Socket for server created" << std::endl;

	addr.sin_addr.s_addr = inet_addr(address);
	addr.sin_port = htons(port_tcp);
	addr.sin_family = AF_INET;

	int ret = bind(server, (SOCKADDR*)&addr, sizeof(addr));
	if (ret < 0)
	{
		std::cout << "SERVER_TCP: Bind error" << WSAGetLastError() << std::endl;
	}

	listen(server, SOMAXCONN);

	std::cout << "SERVER_TCP: Listening client..." << std::endl;

	client = accept(server, (SOCKADDR*)&addr, &sizeofaddr);

	if (client == 0) {
		std::cout << "SERVER_TCP: Error #2\n";
	}
	else {
		std::cout << "SERVER_TCP: Client Connected!\n";
	}
}

int Server_TCP::sendData(const char *buffer, int len)
{
	return send(client, buffer, len, NULL);
}

int Server_TCP::recvData(char *buffer, int len)
{
	return recv(client, buffer, len, NULL);
}

void Server_TCP::close()
{
	closesocket(server);
	WSACleanup();
	std::cout << "Server_TCP is closed" << std::endl;
}

//Server_TCP::~Server_TCP()
//{
//	closesocket(server);
//	WSACleanup();
//	std::cout << "Server_TCP is closed" << std::endl;
//}

SerialPort::SerialPort(const char *portName)
{
	this->connected = false;

	this->handler = CreateFileA(static_cast<LPCSTR>(portName),
		GENERIC_READ | GENERIC_WRITE,
		0,
		NULL,
		OPEN_EXISTING,
		FILE_ATTRIBUTE_NORMAL,
		NULL);
	if (this->handler == INVALID_HANDLE_VALUE) {
		if (GetLastError() == ERROR_FILE_NOT_FOUND) {
			printf("ERROR: Handle was not attached. Reason: %s not available\n", portName);
		}
		else
		{
			printf("ERROR!!!");
		}
	}
	else {
		DCB dcbSerialParameters = { 0 };

		if (!GetCommState(this->handler, &dcbSerialParameters)) {
			printf("failed to get current serial parameters");
		}
		else {
			dcbSerialParameters.BaudRate = CBR_9600;
			dcbSerialParameters.ByteSize = 8;
			dcbSerialParameters.StopBits = ONESTOPBIT;
			dcbSerialParameters.Parity = NOPARITY;
			dcbSerialParameters.fDtrControl = DTR_CONTROL_ENABLE;

			if (!SetCommState(handler, &dcbSerialParameters))
			{
				printf("ALERT: could not set Serial port parameters\n");
			}
			else {
				this->connected = true;
				PurgeComm(this->handler, PURGE_RXCLEAR | PURGE_TXCLEAR);
				Sleep(ARDUINO_WAIT_TIME);
			}
		}
	}
}

SerialPort::~SerialPort()
{
	if (this->connected) {
		this->connected = false;
		CloseHandle(this->handler);
	}
}

int SerialPort::readSerialPort(char *buffer, unsigned int buf_size)
{
	DWORD bytesRead;
	unsigned int toRead = 0;

	ClearCommError(this->handler, &this->errors, &this->status);

	if (this->status.cbInQue > 0) {
		if (this->status.cbInQue > buf_size) {
			toRead = buf_size;
		}
		else toRead = this->status.cbInQue;
	}

	if (ReadFile(this->handler, buffer, toRead, &bytesRead, NULL)) return bytesRead;

	return 0;
}

bool SerialPort::writeSerialPort(char *buffer, unsigned int buf_size)
{
	DWORD bytesSend;

	if (!WriteFile(this->handler, (void*)buffer, buf_size, &bytesSend, 0)) {
		ClearCommError(this->handler, &this->errors, &this->status);
		return false;
	}
	else return true;
}

bool SerialPort::isConnected()
{
	return this->connected;
}