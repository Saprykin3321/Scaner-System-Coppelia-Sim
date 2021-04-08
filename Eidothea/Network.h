#pragma once
#pragma comment(lib, "ws2_32.lib")

#define ARDUINO_WAIT_TIME 2000
#define MAX_DATA_LENGTH 255

#include <stdio.h>
#include <stdlib.h>
#include <winsock2.h>

const char		address[15] = "127.0.0.1";

class Server_UDP
{
public:
	Server_UDP();
	void SendMsg(const char* sendBuf, int size);
	void RecMsg(char *recBuf);
	void close();
	//~Server_UDP();

private:
	SOCKET					m_sServer;
	WSADATA					wsaData;
	struct sockaddr_in		m_SocAddrClient,
							m_SocAddrserver;
	unsigned short			port_udp = 8090;
};

class Server_TCP
{
	WSAData				wsaData;
	SOCKET				server, client;
	SOCKADDR_IN			addr, client1;
	int					sizeofaddr = sizeof(addr);
	char				buf[1024];
	unsigned short		port_tcp = 8091;

public:
	Server_TCP();
	int sendData(const char* buffer, int len);
	int recvData(char* buffer, int len);
	void close();
	//~Server_TCP();
};

class SerialPort
{
private:
	HANDLE handler;
	bool connected;
	COMSTAT status;
	DWORD errors;

public:
	SerialPort(const char *portName);
	~SerialPort();

	int readSerialPort(char *buffer, unsigned int buf_size);
	bool writeSerialPort(char *buffer, unsigned int buf_size);
	bool isConnected();
};