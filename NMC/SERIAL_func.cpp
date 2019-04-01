// serialclass.cpp : 定义 DLL 应用程序的函数。
//设置一个private成员，记录handle值

#include "stdafx.h"
#include<windows.h>
#include<stdio.h>
#include<tchar.h>
#include"SERIAL.h"

#ifndef USCOMM_ERR_PRINTF
#define USCOMM_ERR_PRINTF
#define ErrPrintf(str_err_info) LogPrintf(__LINE__, __FUNCTION__ , #str_err_info)
#endif 

SerialPort::SerialPort()
{
	this->logFileName = "D:\\ComnErr.txt";
	this->errLog.open(logFileName.c_str(), ios::trunc | ios::out);
}

SerialPort::~SerialPort()
{
	Close_driver(this->hcom);
	this->errLog.close();
}

HANDLE SerialPort::Open_driver_Y(TCHAR *name)
{
	//打开串口
	HANDLE m_hCom = CreateFile(
		name,
		GENERIC_READ | GENERIC_WRITE, //读与写
		0,
		NULL,
		OPEN_EXISTING,
		FILE_FLAG_OVERLAPPED,
		NULL);
	if (m_hCom == INVALID_HANDLE_VALUE)
	{
		printf("Create File faile\n");
		ErrPrintf("Create File faile");
		return NULL;
	}

	int buffer_size;
	buffer_size = 1024;
	//设置缓冲区大小
	if (!SetupComm(m_hCom, buffer_size, buffer_size))
	{
		printf("SetupComm fail!\n");
		CloseHandle(m_hCom);
		return NULL;
	}

	//设置超时
	COMMTIMEOUTS TimeOuts;
	memset(&TimeOuts, 0, sizeof(TimeOuts));
	TimeOuts.ReadIntervalTimeout = 100;
	TimeOuts.ReadTotalTimeoutConstant = 1000;
	TimeOuts.ReadTotalTimeoutMultiplier = 100;
	TimeOuts.WriteTotalTimeoutConstant = 2000;
	TimeOuts.WriteTotalTimeoutMultiplier = 50;
	SetCommTimeouts(m_hCom, &TimeOuts);
	PurgeComm(m_hCom, PURGE_TXCLEAR | PURGE_RXCLEAR);//清空缓冲区
												 
	DCB dcb = { 0 };                                 //设置串口参数

	if (!GetCommState(m_hCom, &dcb))
	{
		printf("GetCommState fail\n");
		ErrPrintf("GetCommState fail");
		return NULL;
	}

	dcb.DCBlength = sizeof(dcb);
	if (!BuildCommDCB(TEXT("9600,n,8,1"), &dcb))//填充ＤＣＢ的数据传输率、奇偶校验类型、数据位、停止位
	{
		printf("BuileCOmmDCB fail\n");
		ErrPrintf("BuileCOmmDCB fail");
		CloseHandle(m_hCom);
		return NULL;
	}

	if (SetCommState(m_hCom, &dcb))
	{
		printf("SetCommState OK!\n");
	}

	//建立并初始化异步方式
	ZeroMemory(&wrOverlapped, sizeof(wrOverlapped));

	if (wrOverlapped.hEvent != NULL)
	{
		ResetEvent(wrOverlapped.hEvent);
		wrOverlapped.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
	}
	this->hcom = m_hCom;
	return m_hCom;
}

bool SerialPort::Send_driver_Y(HANDLE fd, BYTE *data, DWORD dwDataLen)
{
	DWORD dwError;
	DWORD dwExpectSend = dwDataLen;
	DWORD dwRealSend = 0;
	BYTE *pSendBuffer;

	pSendBuffer = data;

	if (ClearCommError(fd, &dwError, NULL))
	{
		PurgeComm(fd, PURGE_TXABORT | PURGE_TXCLEAR);
	}

	if (!WriteFile(fd, pSendBuffer, dwExpectSend, &dwRealSend, &wrOverlapped))
	{
		if (GetLastError() == ERROR_IO_PENDING)
		{
			while (!GetOverlappedResult(fd, &wrOverlapped, &dwRealSend, FALSE))
				//成功返回非0，失败返回0
			{
				if (GetLastError() == ERROR_IO_INCOMPLETE)
				{
					continue;
				}
				else
				{
					printf("send Fail!\n");
					ErrPrintf("send fail");
					ClearCommError(fd, &dwError, NULL);
					return true;
				}
			}
		}
	}
	return false;
}

bool SerialPort::Receive_driver_Y(HANDLE fd, BYTE *data)
{
	DWORD dwError;
	DWORD dwWantRead = 20;
	DWORD dwRealRead = 0;
	BYTE* pReadBuf;
	unsigned int i;
	///	BYTE crc;
	pReadBuf = data;
	if (ClearCommError(fd, &dwError, NULL))
	{
		PurgeComm(fd, PURGE_TXABORT | PURGE_TXCLEAR);
	}
	if (!ReadFile(fd, pReadBuf, dwWantRead, &dwRealRead, &wrOverlapped))
	{
		dwError = GetLastError();
		if (dwError == ERROR_IO_PENDING)
		{
			while (!GetOverlappedResult(fd, &wrOverlapped, &dwRealRead, TRUE))
				//成功返回非0, 失败返回0
			{
				return true;
			}
		}
	}
	if (dwRealRead>0) {
		printf("recv_len = %d\n", dwRealRead);
	}
	printf("接收数据\n");

	for (i = 0; i < dwRealRead; i++)
	{
		printf("%.2x ", data[i]);
	}
	printf("\r\n");
	return false;
}

void SerialPort::Close_driver(void *fd)
{
	CloseHandle(fd);
}

void SerialPort::LogPrintf(long line, char *strfunc, char * strErr)
{
	char s[256];

	sprintf_s(s, "%d", line);

	this->errLog << "Date: " << __DATE__ << ".\n";

	this->errLog << "Time: " << __TIME__ << ".\n";

	this->errLog << "Line: " << s << ".\n";

	this->errLog << strfunc << "(...),  " << strErr << ",  pls chk!.\n";

	this->errLog << "\n";

	this->errLog << flush;

}
